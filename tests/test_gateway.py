import hashlib
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as S

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mav

from blueboat_ppo_controller.authority import Authority, GatewayClient, gateway_config
from blueboat_ppo_controller.boat_profile import GCS_CHECKS, initialize, main, report
from blueboat_ppo_controller.ground_owner import GroundOwner
from blueboat_ppo_controller.mavlink_gateway import Gateway, Router
from blueboat_ppo_controller.motor_interface import MotorInterface


def message(value, system=255, component=190):
    encoder = mav.MAVLink(None, srcSystem=system, srcComponent=component)
    return mav.MAVLink(None).parse_buffer(value.pack(encoder))[0]


def heartbeat(system=255, armed=False, mode=0):
    return message(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS if system==255 else mav.MAV_TYPE_SURFACE_BOAT,
        mav.MAV_AUTOPILOT_INVALID if system==255 else mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mav.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0, mode, mav.MAV_STATE_ACTIVE, 3), system, 1)


def version(profile):
    return message(mav.MAVLink_autopilot_version_message(0, profile['flight_sw_version'],0,0,0,[0]*8,[0]*8,[0]*8,0,0,profile['uid']),1,1)




def test_gateway_configuration_resets_approvals_and_keeps_export(gateway_files):
    directory, _ = gateway_files
    profile = json.loads((directory/'motor_interface.json').read_text())
    config = gateway_config(directory/'gateway.json')
    assert profile['gcs_gateway_sha256']==config['config_sha256']
    assert not profile['verified'] and all(not profile[k] for k in GCS_CHECKS)
    assert profile['expected_parameters']['MOT_THST_ASYM']==1.6
    assert not report(directory)['powered_ready']
    assert (directory/'ground.key').stat().st_mode & 0o077 == 0
    key = (directory/'ground.key').read_bytes()
    main(['--directory',str(directory),'gateway-configure','--qgc-ip','127.0.0.2'])
    assert (directory/'ground.key').read_bytes()==key
    assert gateway_config(directory/'gateway.json')['config_sha256']!=config['config_sha256']


def test_motor_interface_supports_verified_sitl_gcs_without_receiver(gateway_files, calibration):
    directory, profile = gateway_files
    motor = MotorInterface(calibration,'sitl')
    assert motor.authority=='joystick_gcs' and motor.enable_channel is None
    profile.update(target='hardware',uid=42,sitl_test_passed=True,restrained_motor_test_passed=True)
    calibration.write_text(json.dumps(profile))
    with pytest.raises(ValueError,match='failsafe tests'): MotorInterface(calibration,'hardware')
    profile.update({k:True for k in GCS_CHECKS})
    calibration.write_text(json.dumps(profile))
    assert MotorInterface(calibration,'hardware').authority=='joystick_gcs'
    config = directory/'gateway.json'
    config.write_text(config.read_text()+'\n')
    with pytest.raises(ValueError,match='checksum'): MotorInterface(calibration,'hardware')


@pytest.mark.parametrize('field,value', [('mavros_system',255),('qgc_peer',['0.0.0.0',14550]),('mavros_peer',['192.168.1.1',14550]),('upstream_peer',['127.0.0.1',-1])])
def test_invalid_gateway_topology_rejected(gateway_files,field,value):
    directory,_=gateway_files
    path=directory/'gateway.json'; config=json.loads(path.read_text());config[field]=value;path.write_text(json.dumps(config))
    with pytest.raises(ValueError): gateway_config(path)


def test_authority_is_explicit_quiet_and_latched():
    a=Authority(True)
    assert not a.request('auto',1)
    a.ground_seen=1; a.vehicle(1,True,0,True); a.last_qgc_control=.5
    assert not a.request('auto',1)
    a.ground_seen=2; a.vehicle(2,True,0,True)
    assert a.request('auto',2)
    token=dict(boot=a.boot,session=a.session,client='one',sequence=1,monotonic_s=2,pwm=[1500,1500])
    assert a.motor(token,2)
    assert not a.motor(token,2.01)
    assert a.reason=='stale_or_replayed_motor_request' and a.actions==['neutral','hold']
    assert not a.request('auto',2.02) and not a.request('manual',2.02)
    a.tick(2.12)
    assert a.actions[-1]=='release'
    a.vehicle(2.13,True,4,True)
    assert a.request('manual',2.13)
    a.vehicle(2.14,True,0,True)
    assert a.request('auto',2.14) and a.session!=token['session']
    assert not a.motor(dict(token,sequence=2,monotonic_s=2.14),2.14)


@pytest.mark.parametrize('failure,reason', [('lease','ground_grant_expired'),('command','controller_request_timeout'),('ground','ground_or_vehicle_stale'),('vehicle','vehicle_mode_disarm_or_fault'),('competing','competing_controller'),('delayed','stale_or_replayed_motor_request')])
def test_authority_failure_cases(failure,reason):
    a=Authority(True); a.ground_seen=1; a.vehicle(1,True,0,True); assert a.request('auto',1)
    packet=dict(boot=a.boot,session=a.session,client='one',sequence=1,monotonic_s=1,pwm=[1550,1500])
    assert a.motor(packet,1)
    if failure=='lease': a.command_seen=1.5; a.tick(1.51)
    elif failure=='command': a.tick(1.21)
    elif failure=='ground': a.ground_seen=-2; a.tick(1.01)
    elif failure=='vehicle': a.vehicle(1.01,True,4,True)
    elif failure=='competing': a.motor(dict(packet,client='two',sequence=2),1.01)
    elif failure=='delayed': a.motor(dict(packet,sequence=2),1.11)
    assert a.state=='stopped' and a.reason==reason
    assert not a.request('auto',1.15)


def test_monitor_never_grants_or_emits_stop_actions():
    a=Authority();a.ground_seen=1;a.vehicle(1,True,0,True)
    for owner in ('auto','manual','stop'): assert not a.request(owner,1)
    a.stop('test',1);a.tick(10)
    assert a.state=='monitor' and not a.actions


def test_router_enforces_origin_identity_and_safe_stop(gateway_files):
    directory,profile=gateway_files
    r=Router(gateway_config(directory/'gateway.json'),profile,True)
    assert r.route('upstream',version(profile),1)
    assert r.route('upstream',heartbeat(1,True),1)
    assert not r.route('mavros',heartbeat(254),1)
    assert r.authority.ground_seen==-float('inf')
    assert not r.route('mavros',heartbeat(),1)
    assert r.route('qgc',heartbeat(),1)
    joystick=message(mav.MAVLink_manual_control_message(1,100,0,0,0,0))
    assert r.route('qgc',joystick,1)
    assert not r.authority.request('auto',1.1)
    assert r.authority.request('auto',2.1)
    assert not r.route('qgc',joystick,2.11)
    assert r.authority.state=='stopped'
    assert r.route('qgc',message(mav.MAVLink_set_mode_message(1,1,4)),2.12)
    assert r.route('qgc',message(mav.MAVLink_command_long_message(1,1,mav.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)),2.12)
    assert not r.route('qgc',message(mav.MAVLink_set_mode_message(1,1,0)),2.12)
    assert not r.route('qgc',heartbeat(),2.12)
    assert not r.route('mavros',message(mav.MAVLink_rc_channels_override_message(1,1,*([1500]*8)),254),2.12)
    assert r.route('mavros',message(mav.MAVLink_param_request_list_message(1,1),254),2.12)
    assert not r.route('mavros',message(mav.MAVLink_param_set_message(1,1,b'RC_OPTIONS',0,9),254),2.12)


def test_vehicle_identity_mismatch_and_staleness_stop(gateway_files):
    directory,profile=gateway_files
    r=Router(gateway_config(directory/'gateway.json'),profile,True)
    r.route('upstream',version(profile),1);r.route('upstream',heartbeat(1,True),1);r.route('qgc',heartbeat(),1)
    assert r.authority.request('auto',1)
    r.route('upstream',version(dict(profile,uid=123456)),1.01)
    assert r.authority.reason=='vehicle_identity_mismatch'
    r=Router(gateway_config(directory/'gateway.json'),profile,True)
    r.route('upstream',version(profile),1);r.route('upstream',heartbeat(1,True),15);r.route('qgc',heartbeat(),15)
    assert r.authority.request('auto',15)
    r.authority.command_seen=16.1;r.authority.grant_until=16.5;r.tick(16.1)
    assert r.authority.reason=='vehicle_identity_stale'


@pytest.mark.parametrize('through_mavproxy', [False, True])
def test_real_udp_and_unix_socket_command_handoff(gateway_files,calibration,through_mavproxy):
    directory,profile=gateway_files
    clients={role:socket.socket(socket.AF_INET,socket.SOCK_DGRAM) for role in ('upstream','qgc','mavros')}
    listeners=[]
    gateway=owner=client=relay=None
    relay_log=None
    errors=[]
    if through_mavproxy:
        script=Path(sys.executable).with_name('mavproxy.py')
        if not script.exists():pytest.skip('Optional actual MAVProxy executable is not installed')
        if any(p.exists() for p in (Path.home()/'.mavinit.scr', Path.home()/'.mavproxy/mavinit.scr')):
            pytest.skip('Refuse to run user MAVProxy startup scripts in an isolated loopback test')
    try:
        config=json.loads((directory/'gateway.json').read_text())
        for role,sock in clients.items():
            sock.bind(('127.0.0.1',0));sock.settimeout(.5);config[role+'_peer']=list(sock.getsockname())
        for role in ('upstream','qgc','mavros','control'):
            sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));listeners.append(sock)
            config[role+'_bind']=list(sock.getsockname())
        vehicle_destination=tuple(config['upstream_bind'])
        if through_mavproxy:
            port=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);port.bind(('127.0.0.1',0));listeners.append(port)
            vehicle_destination=port.getsockname()
            config['upstream_peer']=['127.0.0.1',0]
        for sock in listeners:sock.close()
        path=directory/'gateway.json';path.write_text(json.dumps(config))
        profile['gcs_gateway_sha256']=gateway_config(path)['config_sha256'];calibration.write_text(json.dumps(profile))
        gateway=Gateway(path,profile,True)
        if through_mavproxy:
            relay_log=(directory/'mavproxy.log').open('w+')
            relay=subprocess.Popen([str(script),f'--master=udpin:127.0.0.1:{vehicle_destination[1]}',
                f'--out=udp:127.0.0.1:{config["upstream_bind"][1]}','--nowait','--heartbeat-rate=0',
                '--source-system=253','--default-modules=','--streamrate=-1','--non-interactive','--no-state','--mav20'],
                cwd=directory,stdin=subprocess.DEVNULL,stdout=relay_log,stderr=subprocess.STDOUT)
        def run():
            try:
                while gateway.running: gateway.step(.005)
            except Exception as error:
                errors.append(error)
        thread=threading.Thread(target=run);thread.start()
        clients['qgc'].sendto(heartbeat().get_msgbuf(),tuple(config['qgc_bind']))
        owner=GroundOwner(tuple(config['control_bind']),bytes.fromhex((directory/'ground.key').read_text().strip()))
        deadline=time.monotonic()+1
        while not gateway.router.authority.fresh(time.monotonic()) and time.monotonic()<deadline:
            for value in (version(profile),heartbeat(1,True)):
                clients['upstream'].sendto(value.get_msgbuf(),vehicle_destination)
            if relay: assert relay.poll() is None
            time.sleep(.02)
        assert gateway.router.authority.fresh(time.monotonic())
        assert owner.request('auto')['accepted']
        client=GatewayClient(calibration,MotorInterface(calibration,'sitl'))
        deadline=time.monotonic()+.1
        while not client.granted(time.monotonic()) and time.monotonic()<deadline:time.sleep(.002)
        client.publish([1540,1510])
        received=[]
        deadline=time.monotonic()+.15
        while time.monotonic()<deadline:
            packet=clients['upstream'].recv(65535)
            received.extend(mav.MAVLink(None).parse_buffer(packet))
            if any(m.get_type()=='RC_CHANNELS_OVERRIDE' and m.chan1_raw==1540 for m in received):break
        thrust=[m for m in received if m.get_type()=='RC_CHANNELS_OVERRIDE']
        assert len(thrust)==1 and thrust[0].chan3_raw==1510 and thrust[0].chan2_raw==65535
        assert thrust[0].get_srcSystem()==255
        # A live MAVROS endpoint cannot submit motor commands or renew the ground grant.
        clients['mavros'].sendto(message(mav.MAVLink_rc_channels_override_message(1,1,*([1800]*8)),254).get_msgbuf(),tuple(config['mavros_bind']))
        clients['mavros'].sendto(heartbeat(254).get_msgbuf(),tuple(config['mavros_bind']))
        joystick=message(mav.MAVLink_manual_control_message(1,1000,0,0,0,0))
        clients['qgc'].sendto(joystick.get_msgbuf(),tuple(config['qgc_bind']))
        deadline=time.monotonic()+.4
        while time.monotonic()<deadline:
            packet=clients['upstream'].recv(65535)
            received.extend(mav.MAVLink(None).parse_buffer(packet))
            if any(m.get_type()=='RC_CHANNELS_OVERRIDE' and m.chan1_raw==0 for m in received):break
        assert not any(m.get_type()=='MANUAL_CONTROL' for m in received)
        assert not any(m.get_type()=='RC_CHANNELS_OVERRIDE' and m.chan1_raw==1800 for m in received)
        assert any(m.get_type()=='SET_MODE' and m.custom_mode==4 for m in received)
        assert any(m.get_type()=='RC_CHANNELS_OVERRIDE' and m.chan1_raw==1500 for m in received)
        assert any(m.get_type()=='RC_CHANNELS_OVERRIDE' and m.chan1_raw==0 and m.chan3_raw==0 for m in received)
        assert not any(m.get_type()=='HEARTBEAT' and m.get_srcSystem()!=255 for m in received)
        assert not owner.request('auto')['accepted']
        assert not client.granted(time.monotonic())
        clients['upstream'].sendto(heartbeat(1,True,4).get_msgbuf(),vehicle_destination)
        deadline=time.monotonic()+.2
        while gateway.router.authority.mode!=4 and time.monotonic()<deadline:time.sleep(.005)
        assert owner.request('manual')['accepted']
        assert gateway.router.authority.state=='manual'
    finally:
        if gateway:
            gateway.running=False;thread.join(1);gateway.close()
        if relay:
            relay.terminate()
            try:relay.wait(timeout=3)
            except subprocess.TimeoutExpired:relay.kill();relay.wait(timeout=3)
        if relay_log:relay_log.close()
        if client:client.close()
        if owner:owner.close()
        for sock in clients.values():sock.close()
        for sock in listeners:sock.close()
    assert not errors


def test_ground_loss_with_live_companion_and_renewing_grant(gateway_files):
    directory,profile=gateway_files
    r=Router(gateway_config(directory/'gateway.json'),profile,True)
    r.route('upstream',version(profile),1);r.route('upstream',heartbeat(1,True),1);r.route('qgc',heartbeat(),1)
    assert r.authority.request('auto',1)
    for sequence in range(1,28):
        now=1+sequence*.1
        r.route('upstream',heartbeat(1,True),now)
        assert not r.route('mavros',heartbeat(254),now)
        r.authority.request('auto',now)
        r.authority.motor(dict(boot=r.authority.boot,session=r.authority.session,client='one',sequence=sequence,monotonic_s=now,pwm=[1540,1510]),now)
    assert r.authority.ground_seen==1
    assert r.authority.state=='stopped' and r.authority.reason=='ground_or_vehicle_stale'


def test_grants_require_authentication_and_fresh_single_use_challenge(gateway_files):
    import hmac
    directory,profile=gateway_files
    gateway=Gateway.__new__(Gateway)
    gateway.config=gateway_config(directory/'gateway.json')
    gateway.key=bytes.fromhex((directory/'ground.key').read_text())
    gateway.router=Router(gateway.config,profile,True)
    a=gateway.router.authority;a.ground_seen=1;a.vehicle(1,True,0,True)
    replies=[]
    gateway.sockets={'control':S(sendto=lambda packet,peer:replies.append(json.loads(packet)))}
    gateway.challenges={};gateway.counters={'bad_packet':0}
    peer=('127.0.0.1',49100)
    def grant(now, key, delay=0):
        gateway.control(b'{"operation":"challenge"}',peer,now)
        challenge=replies[-1]
        signed=dict(boot=challenge['boot'],nonce=challenge['nonce'],owner='auto')
        signed['mac']=hmac.new(key,json.dumps(signed,sort_keys=True,separators=(',',':')).encode(),hashlib.sha256).hexdigest()
        packet=json.dumps(signed).encode()
        gateway.control(packet,peer,now+delay)
        return packet
    grant(1,b'wrong'*6)
    assert a.state=='manual'
    grant(1.1,gateway.key,.26)
    assert a.state=='manual'
    packet=grant(1.4,gateway.key)
    assert a.state=='auto' and a.grant_until==1.9
    gateway.control(packet,peer,1.5)
    assert a.grant_until==1.9
    gateway.control(packet,('127.0.0.2',49100),1.51)
    assert a.grant_until==1.9


@pytest.mark.parametrize('action', [0,1,3,4])
def test_motion_or_disabled_fallback_rejected(gateway_files,calibration,action):
    _,profile=gateway_files
    profile['expected_parameters']['FS_ACTION']=action
    calibration.write_text(json.dumps(profile))
    with pytest.raises(ValueError,match='fallback'):MotorInterface(calibration,'sitl')


def test_late_grant_cannot_renew_expired_session():
    a=Authority(True);a.ground_seen=1;a.vehicle(1,True,0,True);assert a.request('auto',1)
    a.command_seen=1.49
    assert not a.request('auto',1.51)
    assert a.state=='stopped' and a.reason=='ground_grant_expired'


def test_operator_stop_commands_pass_even_without_fresh_vehicle_identity(gateway_files):
    directory,profile=gateway_files
    r=Router(gateway_config(directory/'gateway.json'),profile,True)
    assert r.route('qgc',message(mav.MAVLink_set_mode_message(1,1,4)),1)
    assert r.route('qgc',message(mav.MAVLink_command_long_message(1,1,mav.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)),1)
    assert not r.route('qgc',message(mav.MAVLink_set_mode_message(1,1,0)),1)
    assert not r.route('qgc',message(mav.MAVLink_manual_control_message(1,1000,0,0,0,0)),1)


def test_recorded_interface_retains_gateway_location_for_analysis_drafts(gateway_files,calibration):
    import os
    directory,profile=gateway_files
    profile['gcs_gateway_file']=os.path.relpath(directory/'gateway.json',calibration.parent)
    calibration.write_text(json.dumps(profile))
    motor=MotorInterface(calibration,'sitl')
    assert motor.config['gcs_gateway_file']==str((directory/'gateway.json').resolve())
    assert json.loads(calibration.read_text())['gcs_gateway_file']==profile['gcs_gateway_file']
    # Session metadata and generated PPO drafts inherit this absolute local path.
    moved=directory/'moved';moved.mkdir()
    (moved/'profile.json').write_text(json.dumps(motor.config))
    client=GatewayClient(moved/'profile.json',S(config=motor.config))
    try:assert client.config['config_sha256']==profile['gcs_gateway_sha256']
    finally:client.close()
