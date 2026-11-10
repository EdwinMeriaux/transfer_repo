import argparse
import hashlib
import hmac
import json
from pathlib import Path
import socket
import time


class GroundOwner:
    def __init__(self, endpoint, key):
        self.key = key
        if len(key) != 32:
            raise ValueError('Use the 32-byte gateway key')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect(endpoint)
        self.socket.settimeout(.3)


    def request(self, owner):
        self.socket.send(b'{"operation":"challenge"}')
        challenge = json.loads(self.socket.recv(4096))
        signed = dict(boot=challenge['boot'], nonce=challenge['nonce'], owner=owner)
        mac = hmac.new(self.key, json.dumps(signed, sort_keys=True, separators=(',', ':')).encode(), hashlib.sha256).hexdigest()
        self.socket.send(json.dumps(dict(signed, mac=mac)).encode())
        return json.loads(self.socket.recv(4096))


    def close(self):
        self.socket.close()


def main(args=None):
    parser = argparse.ArgumentParser(description='Run on the QGroundControl computer: explicit leased BlueBoat authority')
    parser.add_argument('owner', choices=('auto', 'manual', 'stop'))
    parser.add_argument('--gateway', required=True, help='Gateway IPv4:control-port, normally JETSON_IP:14604')
    parser.add_argument('--key-file', type=Path, required=True)
    options = parser.parse_args(args)
    client = None
    announced = False
    try:
        host, port = options.gateway.rsplit(':', 1)
        socket.inet_pton(socket.AF_INET, host)
        client = GroundOwner((host, int(port)), bytes.fromhex(options.key_file.read_text().strip()))
        while True:
            answer = client.request(options.owner)
            if answer.get('accepted') is not True:
                raise ValueError('Grant refused: '+str(answer.get('reason', answer)))
            if options.owner != 'auto':
                print(json.dumps(answer), flush=True)
                break
            if not announced:
                print('AUTO grant active. Keep this terminal open; Ctrl-C requests stop/HOLD. QGC must remain connected with joystick disabled.', flush=True)
                announced = True
            time.sleep(.1)
    except KeyboardInterrupt:
        print('Stopping ground grant.', flush=True)
    except (ValueError, KeyError, OSError) as error:
        parser.exit(2, str(error)+'\n')
    finally:
        if client:
            if options.owner == 'auto':
                try:
                    client.request('stop')
                except (ValueError, KeyError, OSError):
                    pass  # An unanswered grant expires at the gateway.
            client.close()


if __name__ == '__main__':
    main()
