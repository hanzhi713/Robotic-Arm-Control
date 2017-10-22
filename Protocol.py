import serial
import time

class ServoProtocol:
    cmd_header = [0xcf, 0xfc]

    def __init__(self, port_name, baud_rate=19200):
        self.sr = serial.Serial(port_name, baud_rate)
        time.sleep(2)

    @staticmethod
    def cmd2hex(cmd):
        hex_code = []
        for b in cmd:
            z = hex(b)[2:]
            if b < 0x10:
                z = '0' + z
            hex_code.append(z)
        return " ".join(hex_code)

    def write(self, degrees):
        cmd = ServoProtocol.cmd_header.copy()
        cmd.extend(degrees)
        cmd.append(sum(degrees) % 0xff)
        self.sr.write(serial.to_bytes(cmd))
        return self.sr.read_all() == b'255'

    def close(self):
        self.sr.close()
