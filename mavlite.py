"""
This file is an attempt to produce a bare-minimum Mavlink communication system.
"""
import array
import struct


async def to_string(string: bytes) -> str:
    """
    Get whatever we can from the incoming bytes object.
    """
    try:
        return string.decode("utf-8")
    except (AttributeError, TypeError):
        pass
    result = ""
    try:
        for char in string:
            result2 = result + char
            result2 = result2.encode("ascii", "ignore")
            result = result2
    except (AttributeError, TypeError):
        pass
    return result + "_XXX"


class X25crc:
    """
    Improved CRC code.
    """
    def __init__(self):
        self.crc = 0xffff

    async def create(self, buf=None):
        """
        Perform CRC creation actions independent of class init.
        """
        self.crc = 0xffff
        if buf is not None:
            if isinstance(buf, str):
                await self.accumulate_str(buf)
            else:
                await self.accumulate(buf)
        return self

    async def accumulate(self, buf):
        """
        Add in rolling byte-chunks.
        """
        accum = self.crc
        for b in buf:
            tmp = b ^ (accum & 0xff)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            accum = (accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        self.crc = accum
        return self

    async def accumulate_str(self, buf: [str, bytes]):
        """
        Add in rolling byte-chunks.
        """
        try:
            bytes_array = array.array('B', list(buf))
        except (AttributeError, TypeError):
            bytes_array = array.array('B', list(buf.encode()))
        except (AttributeError, TypeError):
            raise ValueError
        await self.accumulate(bytes_array)
        return self


class Packet:
    """
    Construct/deconstruct Mavlink packets.
    """
    packet = None

    magic = 0xfe  # Start.
    p_len = 0x00  # 0-255  Payload length.
    incompat = 0x00  # 0-255 Incompatibility Flags: https://mavlink.io/en/guide/serialization.html#incompat_flags
    compat = 0x00  # 0-255 Compatibility Flags: https://mavlink.io/en/guide/serialization.html#compat_flags
    psn = 0x00  # 0-255 Packet sequence number.
    sid = 0x01  # 1-255 System ID (sender).
    cid = 0x01  # 1-255 Component ID (sender): https://mavlink.io/en/messages/common.html#MAV_COMPONENT
    mid = 0x00  # 0-16777215 Message ID (low, middle, high bytes)
    payload: bytes = bytes()  # Payload Max 255 bytes: https://mavlink.io/en/guide/serialization.html#payload
    next_payload: bytes = bytes()  # A place to store payload information that is larger than the allowed size.
    crc = 0xffff  # Checksum (low byte, high byte): https://mavlink.io/en/guide/serialization.html#checksum
    sig = 0x00  # Optional signature: https://mavlink.io/en/guide/message_signing.html

    crc_extra = 0x00

    x25 = X25crc()

    def __init__(self):
        self.dummy = None

    async def reset(self):
        """
        Clear all variables to save memory.
        """
        self.p_len = 0x00
        self.incompat = 0x00
        self.compat = 0x00
        self.psn = 0x00
        self.sid = 0x01
        self.cid = 0x01
        self.mid = 0x00
        self.crc = 0xffff
        self.sig = 0x00
        self.payload: bytes = bytes()
        self.crc_extra = 0x00
        self.x25 = X25crc()

        return self

    async def message_checksum(self, msg):
        """
        TODO: If this code doesn't prove better than our original solution, remove it.

        This is example code: https://mavlink.io/en/guide/serialization.html#payload

        calculate an 8-bit checksum of the key fields of a message, so we
        can detect incompatible XML changes.
        """
        crc = self.x25
        await crc.accumulate_str(msg.name + ' ')
        crc_end = msg.base_fields()
        for i in range(crc_end):
            f = msg.ordered_fields[i]
            await crc.accumulate_str(f.type + ' ')
            await crc.accumulate_str(f.name + ' ')
            if f.array_length:
                await crc.accumulate([f.array_length])
        self.crc_extra = (crc.crc & 0xFF) ^ (crc.crc >> 8)
        return self

    async def header(self) -> bytes:
        """
        Constructs a mavlink 2 header.
        """
        result = struct.pack(
            "<BBBBBBBHB",
            253,
            self.p_len,
            self.incompat,
            self.compat,
            self.psn,
            self.sid,
            self.cid,
            self.mid & 0xFFFF,
            self.mid >> 16,
        )
        return result

    async def truncate(self):
        """
        Strip nullbytes from payload.
        """
        if not isinstance(self.payload, bytes):
            raise TypeError
        self.p_len = len(self.payload)
        while self.p_len > 1 and self.payload[self.p_len - 1] == 0:
            self.p_len -= 1
        self.payload = self.payload[:self.p_len]
        return self

    async def assemble(self) -> bytes:
        """
        Assembles a packet for transmission.
        """
        await self.truncate()
        header = await self.header()
        self.packet = header + self.payload
        await self.x25.create(self.packet[1:])
        await self.x25.accumulate_str(struct.pack("B", self.crc_extra))  # noqa
        self.crc = self.x25.crc
        self.packet += struct.pack("<H", self.crc)
        return self.packet
