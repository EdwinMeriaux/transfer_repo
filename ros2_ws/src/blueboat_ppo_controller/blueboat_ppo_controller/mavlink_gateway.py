import argparse
import hashlib
import hmac
import json
import os
from pathlib import Path
import secrets
import selectors
import signal
import socket
import time

from pymavlink.dialects.v20 import ardupilotmega as mav

from .authority import Authority, gateway_config
from .mavros_interface import rc_override

READ_MESSAGES={'PARAM_REQUEST_READ','PARAM_REQUEST_LIST','PARAM_EXT_REQUEST_READ','PARAM_EXT_REQUEST_LIST','PING','TIMESYNC','SYSTEM_TIME'}
READ_COMMANDS={mav.MAV_CMD_REQUEST_MESSAGE,mav.MAV_CMD_SET_MESSAGE_INTERVAL,mav.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES}
MOTOR_MESSAGES={'MANUAL_CONTROL','RC_CHANNELS_OVERRIDE','SET_ACTUATOR_CONTROL_TARGET','SET_ATTITUDE_TARGET',
                'SET_POSITION_TARGET_LOCAL_NED','SET_POSITION_TARGET_GLOBAL_INT'}


class Router:
    def __init__(self, config, profile, enabled=False):
        self.config=config
        self.profile=profile
        self.authority=Authority(enabled)
        self.version_seen=-float('inf')
        self.identity=False
        self.last_version_request=-float('inf')


    def route(self, role, message, now):
        a=self.authority
        self.tick(now)
        kind=message.get_type()
        if kind=='BAD_DATA':
            return False
        if role=='upstream':
            if message.get_srcSystem()!=self.config['vehicle_system']:
                return False
            if message.get_srcComponent()==self.config['vehicle_component']:
                if kind=='AUTOPILOT_VERSION':
                    self.version_seen=now
                    self.identity=(message.uid==self.profile['uid'] and message.flight_sw_version==self.profile['flight_sw_version'])
                    if not self.identity:
                        a.healthy=False
                        a.stop('vehicle_identity_mismatch',now)
                if kind=='HEARTBEAT':
                    healthy=(message.autopilot==mav.MAV_AUTOPILOT_ARDUPILOTMEGA and message.type in (mav.MAV_TYPE_GROUND_ROVER,mav.MAV_TYPE_SURFACE_BOAT)
                             and message.system_status in (mav.MAV_STATE_STANDBY,mav.MAV_STATE_ACTIVE)
                             and self.identity and now-self.version_seen<=15.)
                    a.vehicle(now,bool(message.base_mode&mav.MAV_MODE_FLAG_SAFETY_ARMED),message.custom_mode,healthy)
            return True
        if message.get_srcSystem()!=self.config['gcs_system' if role=='qgc' else 'mavros_system']:
            return False
        target=getattr(message,'target_system',getattr(message,'target',None))
        if target is not None and target not in (0,self.config['vehicle_system']):
            return False
        readonly=kind in READ_MESSAGES or kind in ('COMMAND_LONG','COMMAND_INT') and message.command in READ_COMMANDS
        if role=='mavros':
            # Neither companion heartbeats nor motor packets reach the FC through this endpoint.
            return readonly
        if kind=='HEARTBEAT':
            if message.type!=mav.MAV_TYPE_GCS:
                return False
            a.ground_seen=now
            return a.state!='stopped' or (a.healthy and 0 <= now-a.vehicle_seen <= 2. and (a.mode==4 or not a.armed))
        if readonly:
            return True
        if kind in MOTOR_MESSAGES:
            a.last_qgc_control=now
        if a.state=='auto':
            a.stop('ground_control_takeover',now)
            # Mode/disarm stop requests can pass; joystick motor packets cannot race with autonomy.
        safe_stop=(kind=='SET_MODE' and message.custom_mode==4 and message.base_mode & mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                   or kind=='COMMAND_LONG' and (message.command==mav.MAV_CMD_COMPONENT_ARM_DISARM and message.param1==0
                                               or message.command==mav.MAV_CMD_DO_SET_MODE and message.param1==mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED and message.param2==4))
        if safe_stop:
            return a.enabled
        if a.state=='stopped':
            return False
        return a.enabled and a.state=='manual' and a.fresh(now)


    def tick(self, now):
        self.authority.tick(now)
        if self.authority.state=='auto' and now-self.version_seen>15.:
            self.authority.stop('vehicle_identity_stale',now)


class Gateway:
    def __init__(self, config_path, profile, enabled=False):
        self.config=gateway_config(config_path)
        c=self.config
        self.router=Router(c,profile,enabled)
        self.profile=profile
        self.key=bytes.fromhex(Path(c['key_file']).read_text().strip())
        if len(self.key)!=32 or Path(c['key_file']).stat().st_mode&0o077:
            raise ValueError('Ground-grant key must be 32 random bytes in a private chmod-600 file')
        self.selector=selectors.DefaultSelector()
        self.sockets={}
        self.upstream_peer=tuple(c['upstream_peer']) if c['upstream_peer'][1] else None
        self.challenges={}
        self.encoder=mav.MAVLink(None,srcSystem=c['gcs_system'],srcComponent=c['gateway_component'])
        self.encoder.seq=0
        self.running=True
        self.last_status=-float('inf')
        self.last_request=-float('inf')
        self.counters=dict(forwarded=0,dropped=0,bad_packet=0,motor_requests=0)
        self.log=None
        self.local_bound=False
        self.lock_path=Path(c['socket_file']+'.lock')
        self.lock_path.parent.mkdir(parents=True,exist_ok=True)
        self.lock_fd=os.open(self.lock_path,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
        os.write(self.lock_fd,str(os.getpid()).encode())
        try:
            for role in ('upstream','qgc','mavros','control'):
                sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.sockets[role]=sock
                sock.bind(tuple(c[role+'_bind']))
                sock.setblocking(False)
                self.selector.register(sock,selectors.EVENT_READ,role)
            local=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
            self.sockets['motor']=local
            local.bind(c['socket_file'])
            self.local_bound=True
            os.chmod(c['socket_file'],0o600)
            local.setblocking(False)
            self.selector.register(local,selectors.EVENT_READ,'motor')
            log_path=Path(c['status_file']).with_name(f'gateway-{time.time_ns()}.jsonl')
            self.log=log_path.open('x',buffering=1)
            self.last_reason=None
            self.write_status(time.monotonic())
        except Exception:
            self.close()
            raise


    def send_vehicle(self, message):
        if self.upstream_peer is None:
            return
        packet=message.pack(self.encoder)
        self.encoder.seq=(self.encoder.seq+1)%256
        self.sockets['upstream'].sendto(packet,self.upstream_peer)


    def pwm(self, values):
        channels=[self.profile[side]['rc_channel'] for side in ('left','right')]
        self.send_vehicle(mav.MAVLink_rc_channels_override_message(self.config['vehicle_system'],self.config['vehicle_component'],*rc_override(channels,values)))


    def flush_actions(self):
        a=self.router.authority
        actions,a.actions=a.actions,[]
        for action in actions:
            if not a.enabled:
                continue
            if action=='neutral':
                self.pwm([self.profile[s]['neutral_pwm'] for s in ('left','right')])
            elif action=='release':
                self.pwm(None)
            elif action=='hold':
                self.send_vehicle(mav.MAVLink_set_mode_message(self.config['vehicle_system'],mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4))


    def control(self, packet, peer, now):
        if peer[0]!=self.config['qgc_peer'][0] or len(packet)>4096:
            return
        try:
            data=json.loads(packet)
            if not isinstance(data,dict):
                return
            self.challenges={key:value for key,value in self.challenges.items() if now<=value[0]}
            if data.get('operation')=='challenge':
                if len(self.challenges)>=64:
                    return
                nonce=secrets.token_hex(16)
                self.challenges[nonce]=(now+.25,peer)
                answer=dict(boot=self.router.authority.boot,nonce=nonce,owner=self.router.authority.state,reason=self.router.authority.reason)
            else:
                nonce=data.get('nonce')
                pending=self.challenges.pop(nonce,None)
                signed={key:data.get(key) for key in ('boot','nonce','owner')}
                expected=hmac.new(self.key,json.dumps(signed,sort_keys=True,separators=(',',':')).encode(),hashlib.sha256).hexdigest()
                if pending is None or pending[1]!=peer or now>pending[0] or data.get('boot')!=self.router.authority.boot or not isinstance(data.get('mac'),str) or not hmac.compare_digest(data['mac'],expected):
                    return
                accepted=self.router.authority.request(data.get('owner'),now)
                answer=dict(accepted=accepted,owner=self.router.authority.state,reason=self.router.authority.reason)
            self.sockets['control'].sendto(json.dumps(answer).encode(),peer)
        except (ValueError,TypeError,KeyError):
            self.counters['bad_packet']+=1


    def receive(self, role, packet, peer, now):
        self.router.tick(now)
        if role=='control':
            self.control(packet,peer,now)
            return
        if role=='motor':
            try:
                value=json.loads(packet)
                if not isinstance(value,dict) or len(packet)>4096:
                    raise ValueError('Malformed local motor request')
                if self.router.authority.motor(value,now):
                    pwm=value['pwm']
                    if not isinstance(pwm,list) or len(pwm)!=2:
                        raise ValueError('Invalid motor pair')
                    for side,number in zip(('left','right'),pwm):
                        ch=self.profile[side]['rc_channel']
                        expected=self.profile['expected_parameters']
                        if type(number) is not int or not expected[f'RC{ch}_MIN']<=number<=expected[f'RC{ch}_MAX']:
                            raise ValueError('Motor PWM exceeds verified RC range')
                    self.pwm(pwm)
                    self.counters['motor_requests']+=1
            except (ValueError,TypeError,KeyError):
                self.router.authority.stop('invalid_motor_request',now)
            return
        expected=self.config[role+'_peer']
        if peer[0]!=expected[0] or expected[1] and peer[1]!=expected[1] or role=='upstream' and self.upstream_peer is not None and peer!=self.upstream_peer:
            self.counters['dropped']+=1
            return
        try:
            parser=mav.MAVLink(None)
            messages=parser.parse_buffer(packet) or []
        except mav.MAVError:
            self.counters['bad_packet']+=1
            return
        for message in messages:
            if role=='upstream' and self.upstream_peer is None:
                if message.get_type()!='HEARTBEAT' or message.get_srcSystem()!=self.config['vehicle_system'] or message.get_srcComponent()!=self.config['vehicle_component']:
                    continue
                self.upstream_peer=peer
            if self.router.route(role,message,now):
                if role=='upstream':
                    for client in ('qgc','mavros'):
                        self.sockets[client].sendto(message.get_msgbuf(),tuple(self.config[client+'_peer']))
                elif self.upstream_peer:
                    self.sockets['upstream'].sendto(message.get_msgbuf(),self.upstream_peer)
                self.counters['forwarded']+=1
            else:
                self.counters['dropped']+=1


    def write_status(self, now):
        state=self.router.authority.status(now,self.config['config_sha256'])
        state['counters']=self.counters
        path=Path(self.config['status_file'])
        temporary=path.with_suffix('.tmp')
        temporary.write_text(json.dumps(state,allow_nan=False))
        os.chmod(temporary,0o600)
        temporary.replace(path)
        if self.log and state['reason']!=self.last_reason:
            self.log.write(json.dumps({k:v for k,v in state.items() if k not in ('boot','session')})+'\n')
            self.last_reason=state['reason']
        self.last_status=now


    def step(self, timeout=.01):
        for key,_ in self.selector.select(timeout):
            for _ in range(32):
                try:
                    packet,peer=key.fileobj.recvfrom(65535)
                except BlockingIOError:
                    break
                self.receive(key.data,packet,peer,time.monotonic())
        now=time.monotonic()
        self.router.tick(now)
        if self.upstream_peer and now-self.last_request>5.:
            self.send_vehicle(mav.MAVLink_command_long_message(self.config['vehicle_system'],self.config['vehicle_component'],
                mav.MAV_CMD_REQUEST_MESSAGE,0,mav.MAVLINK_MSG_ID_AUTOPILOT_VERSION,0,0,0,0,0,0))
            self.last_request=now
        self.flush_actions()
        if now-self.last_status>=.02:
            self.write_status(now)


    def close(self):
        try:
            if 'upstream' in self.sockets and self.local_bound:
                self.router.authority.stop('gateway_shutdown',time.monotonic())
                self.flush_actions()
                if self.router.authority.enabled:
                    self.pwm(None)
                self.write_status(time.monotonic())
        finally:
            for sock in self.sockets.values():
                sock.close()
            self.selector.close()
            if self.log:
                self.log.close()
            if self.local_bound:
                Path(self.config['socket_file']).unlink(missing_ok=True)
            os.close(self.lock_fd)
            self.lock_path.unlink(missing_ok=True)


def main():
    parser=argparse.ArgumentParser(description='Exclusive QGC / BlueBoat motor gateway upstream of one MAVProxy endpoint')
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--interface',type=Path,required=True)
    parser.add_argument('--enable-forwarding',action='store_true')
    args=parser.parse_args()
    profile=json.loads(args.interface.read_text())
    config=gateway_config(args.config)
    if args.enable_forwarding:
        from .motor_interface import MotorInterface
        motor=MotorInterface(args.interface,profile['target'])
        if motor.authority!='joystick_gcs' or config['config_sha256']!=profile.get('gcs_gateway_sha256'):
            parser.error('Forwarding requires a verified GCS profile bound to this exact gateway config')
    gateway=Gateway(args.config,profile,args.enable_forwarding)
    def stop(signum, frame):
        gateway.running=False
    handlers={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        while gateway.running:
            gateway.step()
    finally:
        gateway.close()
        for sig,handler in handlers.items():
            signal.signal(sig,handler)


if __name__ == '__main__':
    main()
