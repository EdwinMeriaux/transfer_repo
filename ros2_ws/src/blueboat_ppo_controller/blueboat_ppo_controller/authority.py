import hashlib
import json
import math
from pathlib import Path
import secrets
import socket
import time


def gateway_config(path):
    path = Path(path).resolve()
    c = json.loads(path.read_text())
    if c.get('schema') != 1:
        raise ValueError('Unknown gateway configuration schema')
    for name in ('upstream_bind','upstream_peer','qgc_bind','qgc_peer','mavros_bind','mavros_peer','control_bind'):
        value = c.get(name)
        if not isinstance(value,list) or len(value)!=2 or not isinstance(value[0],str) or type(value[1]) is not int:
            raise ValueError('Configure an IPv4 [address, port]: '+name)
        socket.inet_pton(socket.AF_INET, value[0])
        if not (0 if name=='upstream_peer' else 1) <= value[1] <= 65535:
            raise ValueError('Invalid port: '+name)
    if c['qgc_peer'][0] in ('0.0.0.0','255.255.255.255') or c['upstream_peer'][0]=='0.0.0.0':
        raise ValueError('Peers must be explicit hosts, not wildcards')
    if c['mavros_peer'][0]!='127.0.0.1' or c['mavros_bind'][0]!='127.0.0.1':
        raise ValueError('MAVROS and gateway must share this host; use the loopback endpoint')
    binds=[tuple(c[k]) for k in ('upstream_bind','qgc_bind','mavros_bind','control_bind')]
    if len(set(binds))!=4 or len({port for _,port in binds})!=4:
        raise ValueError('Use distinct gateway listening ports')
    for name in ('vehicle_system','vehicle_component','gcs_system','gateway_component','mavros_system'):
        if type(c.get(name)) is not int or not 1<=c[name]<=255:
            raise ValueError('Invalid MAVLink ID: '+name)
    if len({c['vehicle_system'], c['gcs_system'], c['mavros_system']}) != 3:
        raise ValueError('Vehicle, GCS and MAVROS system IDs must differ')
    for name in ('socket_file','status_file','key_file'):
        if not isinstance(c.get(name),str) or not c[name]:
            raise ValueError('Missing local gateway path: '+name)
        c[name]=str((path.parent/c[name]).resolve())
    if len(c['socket_file'].encode())>100 or len({c[k] for k in ('socket_file','status_file','key_file')})!=3:
        raise ValueError('Use short, distinct local socket/status/key paths')
    c['config_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    return c


class Authority:
    def __init__(self, enabled=False):
        self.enabled=enabled
        self.state='manual' if enabled else 'monitor'
        self.boot=secrets.token_hex(16)
        self.session=None
        self.client=None
        self.sequence=-1
        self.ground_seen=self.vehicle_seen=self.last_qgc_control=-math.inf
        self.grant_until=self.command_seen=-math.inf
        self.armed=False
        self.mode=None
        self.healthy=False
        self.reason='startup'
        self.actions=[]
        self.stopped_at=None
        self.released=False
        self.last_hold=-math.inf


    def fresh(self, now):
        return 0<=now-self.ground_seen<=2.5 and 0<=now-self.vehicle_seen<=2. and self.healthy


    def vehicle(self, now, armed, mode, healthy):
        self.vehicle_seen=now
        self.armed, self.mode, self.healthy=armed, mode, healthy
        if self.state=='auto' and (not armed or mode!=0 or not healthy):
            self.stop('vehicle_mode_disarm_or_fault',now)


    def request(self, owner, now):
        self.tick(now)
        if not self.enabled:
            return False
        if owner=='stop':
            self.stop('operator_stop',now)
            return True
        if owner=='manual':
            if self.state=='auto':
                self.stop('manual_requested_wait_for_hold',now)
                return False
            if self.state=='stopped' and not (self.fresh(now) and (not self.armed or self.mode==4) and self.released):
                return False
            self.state='manual'
            self.reason='manual_granted'
            self.session=self.client=None
            return True
        if owner!='auto' or self.state not in ('manual','auto') or not self.fresh(now) or not self.armed or self.mode!=0:
            return False
        if self.state=='manual':
            if now-self.last_qgc_control<1.:
                return False
            self.state='auto'
            self.session=secrets.token_hex(16)
            self.client=None
            self.sequence=-1
            self.command_seen=now
            self.reason='ground_granted'
        self.grant_until=now+.5
        return True


    def stop(self, reason, now):
        if not self.enabled or self.state=='stopped':
            return
        self.state='stopped'
        self.reason=reason
        self.session=None
        self.grant_until=-math.inf
        self.stopped_at=now
        self.released=False
        self.last_hold=now
        self.actions.extend(['neutral','hold'])


    def tick(self, now):
        if self.state=='auto':
            if not self.fresh(now):
                self.stop('ground_or_vehicle_stale',now)
            elif now>self.grant_until:
                self.stop('ground_grant_expired',now)
            elif now-self.command_seen>.2:
                self.stop('controller_request_timeout',now)
        if self.state=='stopped':
            if not self.released and now-self.stopped_at>=.1:
                self.actions.append('release')
                self.released=True
            if (self.armed and self.mode!=4 or now-self.vehicle_seen>2.) and now-self.last_hold>=.2:
                self.actions.append('hold')
                self.last_hold=now


    def motor(self, payload, now):
        self.tick(now)
        if self.state!='auto' or payload.get('boot')!=self.boot or payload.get('session')!=self.session:
            return False
        sent=payload.get('monotonic_s')
        sequence=payload.get('sequence')
        client=payload.get('client')
        if not isinstance(sent,(int,float)) or not 0<=now-sent<=.1 or type(sequence) is not int or sequence<=self.sequence:
            self.stop('stale_or_replayed_motor_request',now)
            return False
        if not isinstance(client,str) or not client or (self.client is not None and client!=self.client):
            self.stop('competing_controller',now)
            return False
        self.client=client
        self.sequence=sequence
        self.command_seen=now
        if payload.get('pwm') is None:
            self.stop('controller_release',now)
            return False
        return True


    def status(self, now, config_sha):
        self.tick(now)
        return dict(schema=1,boot=self.boot,session=self.session,owner=self.state,reason=self.reason,
                    monotonic_s=now,config_sha256=config_sha,client=self.client,
                    grant_remaining_s=max(0.,self.grant_until-now),ground_fresh=0<=now-self.ground_seen<=2.5,
                    vehicle_fresh=0<=now-self.vehicle_seen<=2.,armed=self.armed,mode=self.mode)


class GatewayClient:
    def __init__(self, profile_path, motor):
        config_path=Path(profile_path).parent/motor.config['gcs_gateway_file']
        self.config=gateway_config(config_path)
        self.client=secrets.token_hex(16)
        self.sequence=0
        self.grant=None
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.socket.setblocking(False)


    def granted(self, now):
        try:
            value=json.loads(Path(self.config['status_file']).read_text())
            now=max(now,time.monotonic())
            valid=(value.get('schema')==1 and value.get('config_sha256')==self.config['config_sha256']
                   and value.get('owner')=='auto' and 0<=now-value['monotonic_s']<=.15
                   and value.get('ground_fresh') is True and value.get('vehicle_fresh') is True
                   and value.get('client') in (None,self.client) and value.get('grant_remaining_s',0)>0)
            if valid:
                self.grant=value
            return valid
        except (ValueError,KeyError,TypeError,OSError):
            return False


    def publish(self, pwm):
        now=time.monotonic()
        if not self.granted(now):
            if pwm is None:
                return
            raise ValueError('No fresh exclusive ground grant')
        self.sequence+=1
        payload=dict(boot=self.grant['boot'],session=self.grant['session'],client=self.client,
                     sequence=self.sequence,monotonic_s=now,pwm=pwm)
        self.socket.sendto(json.dumps(payload,allow_nan=False).encode(),self.config['socket_file'])


    def close(self):
        self.socket.close()
