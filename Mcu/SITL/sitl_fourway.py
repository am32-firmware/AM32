'''
4-way / 1-wire bootloader protocol client for the AM32 SITL bootloader.

Talks the direct 19200 baud one-wire protocol over the SITL input UDP
port (type 4 serial packets, see sitl_dshot.py). Frame formats follow
bootloader/main.c in the am32-bootloader repo:

  commands are fixed length plus a 16 bit CRC (poly 0xA001 reflected,
  little endian on the wire):
    CMD_SET_ADDRESS (0xFF): FF 00 ADDR_HI ADDR_LO crc crc     -> ACK
    CMD_SET_BUFFER  (0xFE): FE 00 HI256 SIZE crc crc          (no ack)
        then SIZE payload bytes + crc crc                     -> ACK
    CMD_PROG_FLASH  (0x01): 01 00 crc crc                     -> ACK
    CMD_READ_FLASH  (0x03): 03 SIZE crc crc  -> SIZE bytes + crc crc + ACK
    CMD_KEEP_ALIVE  (0xFD): FD 00 crc crc                     -> 0xC1
    CMD_RUN         (0x00): 00 00 00 00                       (jump)
  device info probe: 17+ zero bytes with [8]=13 [9]=66 [16]=0x7d
        -> 9 byte deviceInfo ending in 0x30

ACK values: 0x30 good, 0xC1 bad command, 0xC2 bad CRC.

magic addresses for CMD_SET_ADDRESS (protocol v2+):
  0x20 eeprom, 0x21 filename, 0x22 continue-from-last, 0x23 devinfo
'''

import struct
import time

import sitl_dshot as sd

ACK_OK = 0x30
ACK_BAD_CMD = 0xC1
ACK_BAD_CRC = 0xC2

CMD_RUN = 0x00
CMD_PROG_FLASH = 0x01
CMD_ERASE_FLASH = 0x02
CMD_READ_FLASH_SIL = 0x03
CMD_KEEP_ALIVE = 0xFD
CMD_SET_BUFFER = 0xFE
CMD_SET_ADDRESS = 0xFF

ADDR_MAGIC_EEPROM = 0x20
ADDR_MAGIC_FILE_NAME = 0x21
ADDR_MAGIC_CONTINUE = 0x22
ADDR_MAGIC_DEVINFO = 0x23


def crc16(data):
    '''bootloader CRC16, poly 0xA001 reflected'''
    ret = 0
    for b in data:
        for _ in range(8):
            if (b ^ ret) & 1:
                ret = (ret >> 1) ^ 0xA001
            else:
                ret >>= 1
            b >>= 1
    return ret & 0xFFFF


def with_crc(frame):
    return frame + struct.pack('<H', crc16(frame))


class FourWay(object):
    '''4-way protocol session over a sitl_dshot.InputPort'''

    def __init__(self, port=None, host='127.0.0.1', udp_port=57733):
        self.port = port or sd.InputPort(host, udp_port)
        self.device_info = None
        self.address_shift = 0

    def close(self):
        self.port.close()

    def _txn(self, frame, reply_len, timeout=2.0):
        '''send a frame and wait for reply_len reply bytes'''
        self.port.flush_serial()
        self.port.send_serial(frame)
        return self.port.read_serial(reply_len, timeout=timeout)

    def connect(self, timeout=2.0):
        '''device info probe; returns the 9 deviceInfo bytes or None'''
        probe = bytearray(17)
        probe[8] = 13
        probe[9] = 66
        probe[16] = 0x7d
        r = self._txn(bytes(probe), 9, timeout=timeout)
        if len(r) == 9 and r[0:3] == b'471' and r[8] == ACK_OK:
            self.device_info = r
            return r
        return None

    def read_devinfo_struct(self, timeout=2.0):
        '''protocol v3: read the devinfo structure via the magic address.
        returns dict or None. Also learns address_shift for later
        set_address calls'''
        if not self.set_address(ADDR_MAGIC_DEVINFO):
            return None
        data = self.read_flash(27, set_addr=False, timeout=timeout)
        if data is None or len(data) < 27:
            return None
        magic1, magic2 = struct.unpack('<II', data[0:8])
        info = {
            'deviceInfo': data[8:17],
            'length': data[17],
            'address_shift': data[18],
            'firmware_start': struct.unpack('<H', data[19:21])[0],
            'filename_start': struct.unpack('<H', data[21:23])[0],
            'eeprom_start': struct.unpack('<H', data[23:25])[0],
            'tune_start': struct.unpack('<H', data[25:27])[0],
            'magic_ok': (magic1, magic2),
        }
        self.address_shift = info['address_shift']
        return info

    def _ack(self, timeout=2.0):
        r = self.port.read_serial(1, timeout=timeout)
        return r[0] if len(r) == 1 else None

    def set_address(self, addr16, timeout=2.0):
        frame = with_crc(struct.pack('>BBH', CMD_SET_ADDRESS, 0, addr16))
        self.port.flush_serial()
        self.port.send_serial(frame)
        return self._ack(timeout) == ACK_OK

    def set_buffer(self, payload, timeout=2.0):
        '''CMD_SET_BUFFER + payload upload (no ack after the command
        itself, one ack after the payload)'''
        size = len(payload)
        assert 1 <= size <= 256
        hi = 1 if size == 256 else 0
        lo = 0 if size == 256 else size
        self.port.flush_serial()
        self.port.send_serial(with_crc(bytes([CMD_SET_BUFFER, 0, hi, lo])))
        # the payload is a separate frame: a line idle gap ends the
        # command frame first, as the inter-command latency does on a
        # real serial adapter
        self.port.send_serial(with_crc(payload), gap=True)
        return self._ack(timeout) == ACK_OK

    def prog_flash(self, timeout=4.0):
        self.port.flush_serial()
        self.port.send_serial(with_crc(bytes([CMD_PROG_FLASH, 0])))
        return self._ack(timeout) == ACK_OK

    def write(self, addr16, payload, timeout=4.0):
        '''write payload at the (16 bit, shifted) protocol address'''
        if not self.set_address(addr16):
            return False
        if not self.set_buffer(payload):
            return False
        return self.prog_flash(timeout=timeout)

    def read_flash(self, size, addr16=None, set_addr=True, timeout=4.0):
        '''read size bytes (1..256, 256 sent as 0) from the current or
        given protocol address. Returns bytes or None on CRC error'''
        assert 1 <= size <= 256
        if set_addr and addr16 is not None:
            if not self.set_address(addr16):
                return None
        self.port.flush_serial()
        self.port.send_serial(with_crc(bytes([CMD_READ_FLASH_SIL, size & 0xFF if size < 256 else 0])))
        r = self.port.read_serial(size + 3, timeout=timeout)
        if len(r) != size + 3 or r[size + 2] != ACK_OK:
            return None
        if struct.unpack('<H', r[size:size + 2])[0] != crc16(r[:size]):
            return None
        return r[:size]

    def keep_alive(self, timeout=2.0):
        self.port.flush_serial()
        self.port.send_serial(with_crc(bytes([CMD_KEEP_ALIVE, 0])))
        return self._ack(timeout) == ACK_BAD_CMD  # 0xC1 by protocol

    def run(self):
        '''CMD_RUN: ask the bootloader to jump to the application'''
        self.port.send_serial(bytes([CMD_RUN, 0, 0, 0]))
