"""
This file is an attempt to produce a bare-minimum Mavlink UART communication system.

"""
import array
import struct
from MSGFormats import formats
from uart import (
    uart_io,
    read_buffer,
    write_buffer,
)


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

    magic = 0xfd  # Start.
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

    header = bytes()

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
        self.header = bytes()
        self.payload = bytes()

        return self

    async def create_header(self):
        """
        Constructs a mavlink 2 header.
        """
        self.header = struct.pack(
            "<BBBBBBBHB",
            self.magic,
            self.p_len,
            self.incompat,
            self.compat,
            self.psn,
            self.sid,
            self.cid,
            self.mid & 0xFFFF,
            self.mid >> 16,
        )
        return self

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

    async def assemble(self):
        """
        Assembles a packet for transmission.
        """
        await self.create_payload(self.mid, self.payload)
        await self.create_header()
        self.packet = self.header + self.payload
        await self.x25.create(self.packet[1:])
        await self.x25.accumulate_str(struct.pack("B", self.crc_extra))  # noqa
        self.crc = self.x25.crc
        self.packet += struct.pack("<H", self.crc)
        return self

    async def create_payload(self, message_id, payload):
        """
        Pack the user payload according to the specified format.
        """
        _format = formats[message_id]
        if len(payload) != len(_format):
            print('payload malformed, expected arguments:', len(_format), 'arguments received:', len(payload))
            raise ValueError
        _format = "<" + formats[message_id]
        self.payload = struct.pack(_format, *payload)
        await self.truncate()
        self.p_len = len(list(self.payload))
        return self

    async def create_packet(
            self,
            m_id: int,
            payload: [list, tuple],
            c_flags: int = 0x00,
            i_flags: int = 0x00,
            s_id: int = 0x01,
            c_id: int = 0x01,

    ) -> bytes:
        """
        Creates a finished mavlink packet that is ready for transmission.
        """
        await self.reset()
        self.mid = m_id
        self.cid = c_id
        self.sid = s_id
        self.payload = payload
        self.compat = c_flags
        self.incompat = i_flags
        await self.assemble()
        return self.packet

    async def send(self):
        """
        This will transmit our shiny new packet.
        """
        if self.packet:
            write_buffer.append(self.packet)
            await self.reset()
        return self

    @staticmethod
    async def receive():
        """
        Just returns the read buffer.
        """
        return read_buffer


class Heartbeat:
    """
    Send a heartbeat packet.
    """

    packet = Packet()

    def __init__(self):
        self.type = None

    @staticmethod
    async def wait() -> True:
        """
        Waits for a heartbeat.
        """
        beat = False
        while not beat:
            for idx, message in enumerate(read_buffer):
                if message['message_id'] == 0:
                    beat = True
                    del read_buffer[idx]
                    break
        return True


async def decode_payload(message_id: int, payload: list, debug: bool = False):
    """
    Uses the indexed format to decode the contents of an incoming payload.
    """
    _format = "<" + formats[message_id]
    p_len = struct.calcsize(_format)
    if p_len > len(payload):
        diff = p_len - len(payload)
        suffix = [0] * diff
        payload.extend(suffix)
    if debug:
        print('using format:', _format)
        print('payload length:', len(payload), 'payload', *payload)
    payload = struct.unpack(_format, bytes(payload))
    return payload


class MavLink:
    """
    This is where it all comes together babah.
    """
    def __init__(self, message_ids: list):
        for f in formats:
            if f not in message_ids:
                del formats[f]
        self.message_ids = message_ids
        self.heartbeat = Heartbeat()
        self.packet = Packet()

    async def heartbeat_wait(self):
        """
        Wait for heartbeat packet.
        """
        return self.heartbeat.wait()

    async def create_message(
            self,
            m_id: int,
            payload: [list, tuple],
            c_flags: int = 0x00,
            i_flags: int = 0x00,
            s_id: int = 0x01,
            c_id: int = 0x01,
    ) -> bytes:
        """
        Formats an outgoing message into a Mavlink packet.
        """
        return await self.packet.create_packet(
            m_id,
            payload,
            c_flags,
            i_flags,
            s_id,
            c_id
        )

    async def send(self):
        """
        Transmits our data.
        """
        await self.packet.send()
        return self

    async def receive(self):
        """
        Read from the incoming buffer
        """
        result = await self.packet.receive()
        return result


async def test(_uart):
    """
    Test
    """
    m_id = 111
    m = MavLink([m_id])
    pk = await m.create_message(
        m_id=111,
        payload=[0, 10000],
        c_flags=0,
        i_flags=0,
        s_id=1,
        c_id=1
    )
    p = list(pk)
    pay_end = 10 + p[1]
    pl = await decode_payload(m_id, p[10:pay_end], debug=True)
    msg = f'start {p[0]}, length {p[1]}, incompat {p[2]}, compat {p[3]}, seq {p[4]}, sys_id {p[5]}, '
    msg += f'comp_id {p[6]}, mes_id {struct.unpack("H", bytes(p[7:9]))[0]}, '
    msg += f'payload {pl}, chk {p[pay_end:pay_end + 2]}'
    read_buffer.append({
        'message_id': struct.unpack("H", bytes(p[7:9]))[0],
        'system_id': p[5],
        'component_id': p[6],
        'payload': p[10:pay_end]
    })
    print(msg)
    await m.send()
    await uart_io(_uart, True)
