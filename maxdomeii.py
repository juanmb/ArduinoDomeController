import struct
import logging
import time
import serial

PORT = '/dev/ttyUSB0'

CMD_ABORT =   0x03
CMD_HOME =    0x04
CMD_GOTO =    0x05
CMD_SHUTTER = 0x06
CMD_STATUS =  0x07
CMD_TICKS =   0x09
CMD_ACK =     0x0A
CMD_SETPARK = 0x0B
CMD_VBAT =    0x0C



def to_hex(data):
    return ' '.join(['%02X' % ord(c) for c in data])


class MaxDomeII(object):
    def __init__(self, port):
        self.__ser = serial.Serial(port, 19200, timeout=1)
        #time.sleep(2)

    def __calc_crc(self, data):
        crc = 0
        for byte in data:
            crc = 0xff & (crc - ord(byte))
        return crc

    def __check_crc(self, packet):
        crc = self.__calc_crc(packet[1:-1])
        if crc != ord(packet[-1]):
            raise ValueError("CRC error")

    def __make_packet(self, cmd_id, payload=''):
        msg = struct.pack('BB%ds' % len(payload), len(payload) + 2, cmd_id, payload)
        return struct.pack('B%dsB' % len(msg), 1, msg, self.__calc_crc(msg))

    def __send(self, cmd_id, resp_length, payload=''):
        packet = self.__make_packet(cmd_id, payload)
        self.__ser.write(packet)
        logging.debug("SENT: %s" % to_hex(packet))

        resp = self.__ser.read(resp_length)
        logging.debug("RECV: %s" % to_hex(resp))

        if ord(resp[0]) != 0x01:
            raise ValueError("Invalid packet start byte: 0x%0x" % ord(resp[0]))
        self.__check_crc(resp)
        return resp

    def send_ack(self):
        self.__send(CMD_ACK, 5)

    def home_azimuth(self):
        self.__send(CMD_HOME, 5)

    def goto_azimuth(self, pos, ccw=False):
        _dir = 1 if ccw else 2
        self.__send(CMD_GOTO, 5, struct.pack('!bH', _dir, pos))

    def abort(self):
        self.__send(CMD_ABORT, 5)

    def get_status(self):
        resp = self.__send(CMD_STATUS, 11)
        sh_st, az_st, az_pos, home_pos = struct.unpack('!bbHH', resp[3:9])

        return {
            'shutter_status': sh_st,
            'azimuth_status': az_st,
            'azimuth_pos': az_pos,
            'home_pos': home_pos,
        }

    def get_voltage(self):
        resp = self.__send(CMD_VBAT, 6)
        vbat = struct.unpack('!H', resp[3:5])[0]
        return vbat*5.0/1023.0 + 8.4


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    dome = MaxDomeII(PORT)

    dome.send_ack()
    print dome.get_status()
    dome.goto_azimuth(40, ccw=True)
    #dome.home_azimuth()
    time.sleep(1)
    dome.abort()
    #print dome.get_voltage()
