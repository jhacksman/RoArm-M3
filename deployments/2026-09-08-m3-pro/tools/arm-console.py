"""Persistent serial console, selecting arms by inventoried USB serial and MAC.

Run with the project Python and one or more labels. Send JSON lines on stdin:
{"arm":"L1","commands":[{"T":602}],"wait":3}
{"arm":"L1","capture":5}
{"quit":true}
All received output is archived in the arm's backup directory. Opening a serial
port can reset some USB adapters; support the arms before starting this tool.
"""
import collections
import json
from pathlib import Path
import re
import sys
import threading
import time
import termios

import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parent.parent
inventory = json.loads((ROOT / 'arms.json').read_text())


class Arm:
    def __init__(self, label):
        self.label = label
        self.info = inventory[label]
        ports = [p.device for p in list_ports.comports()
                 if p.serial_number == self.info['usb_serial']]
        if len(ports) != 1:
            raise RuntimeError(f'{label}: expected one matching USB device, got {ports}')
        self.s = serial.Serial(port=None, baudrate=115200, timeout=0.1)
        self.s.dtr = False
        self.s.rts = False
        self.s.port = ports[0]
        self.s.open()
        attributes = termios.tcgetattr(self.s.fileno())
        attributes[2] &= ~termios.HUPCL
        termios.tcsetattr(self.s.fileno(), termios.TCSANOW, attributes)
        self.chunks = collections.deque(maxlen=2000)
        self.lock = threading.Lock()
        self.running = True
        self.error = None
        self.log = (ROOT / self.info['backup_directory'] / 'pairing-session.log').open('a')
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        try:
            while self.running:
                data = self.s.read(4096)
                if data:
                    message = data.decode(errors='replace')
                    with self.lock:
                        self.chunks.append((time.monotonic(), message))
                        self.log.write(message)
                        self.log.flush()
        except Exception as exc:
            self.error = str(exc)

    def since(self, start):
        if self.error:
            raise RuntimeError(self.error)
        with self.lock:
            return ''.join(s for t, s in self.chunks if t >= start)

    def send(self, command, delay):
        payload = json.dumps(command, separators=(',', ':')) + '\n'
        if len(payload.encode()) > 255:
            raise ValueError('Command exceeds firmware input buffer')
        start = time.monotonic()
        self.s.write(payload.encode())
        self.s.flush()
        time.sleep(delay)
        return self.since(start)

    def verify_identity(self):
        output = self.send({'T': 302}, 3)
        macs = re.findall(r'(?im)^([0-9a-f]{2}(?::[0-9a-f]{2}){5})\s*$', output)
        if self.info['mac'].lower() not in [m.lower() for m in macs]:
            raise RuntimeError(f'{self.label}: MAC identity check failed; see session log')

    def close(self):
        self.running = False
        time.sleep(0.2)
        self.s.close()
        self.log.close()


arms = {}
try:
    for label in sys.argv[1:]:
        arms[label] = Arm(label)
    time.sleep(20)
    for arm in arms.values():
        arm.verify_identity()
    print(json.dumps({'ready': {k: v.info['mac'] for k, v in arms.items()}}), flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            if request.get('quit'):
                break
            arm = arms[request['arm']]
            if 'capture' in request:
                start = time.monotonic()
                time.sleep(min(float(request['capture']), 30))
                responses = [arm.since(start)]
            else:
                delay = min(float(request.get('wait', 2)), 30)
                responses = [arm.send(cmd, delay) for cmd in request['commands']]
            print(json.dumps({'arm': arm.label, 'responses': responses}), flush=True)
        except Exception as exc:
            print(json.dumps({'error': str(exc)}), flush=True)
finally:
    for arm in arms.values():
        arm.close()
