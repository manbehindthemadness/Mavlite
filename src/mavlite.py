"""
This file is an attempt to produce a bare-minimum Mavlink UART communication system.
https://docs.python.org/3/library/struct.html

TypesList = {
    "float":"f",
    "double":"d",
    "char":"s",
    "int8_t":"b",
    "int16_t":"h",
    "int32_t":"l",
    "int64_t":"q",
    "uint8_t":"B",
    "uint16_t":"H",
    "uint32_t":"I",
    "uint64_t":"Q"
    }
"""
import gc
import array
import struct
from time import monotonic
try:
    import asyncio
except ImportError:
    import uasyncio as asyncio  # noqa

try:
    from MSGFormats import formats
    from uart import (
        uart_read,
        uart_write,
        write_buffer,
    )
except ImportError:
    from .MSGFormats import formats
    from .uart import (
        uart_read,
        uart_write,
        write_buffer,
    )

TERM = False
read_buffer = list()

defs = {
    'sid': 1,
    'cid': 158,
    'cfl': 0,
    'ifl': 0

}


async def decode_payload(message_id: int, payload: list, format_override: [None, str] = None, debug: bool = False):
    """
    Uses the indexed format to decode the contents of an incoming payload.
    """
    if not format_override:
        _format = "<" + formats[message_id][0]
    else:
        _format = "<" + format_override
    p_len = struct.calcsize(_format)
    if p_len > len(payload):
        diff = p_len - len(payload)
        suffix = [0] * diff  # Pad to fit.
        payload.extend(suffix)
    if debug:
        print('using format:', _format)
        print('payload length:', len(payload), 'payload', *payload)
    payload = struct.unpack(_format, bytes(payload))
    # Decode Order
    if message_id <= 255:
        tmpPayload = payload[:]
        for i in range(0, len(payload)):
            payload[i] = tmpPayload[formats[message_id][2][i]]  # noqa
            await asyncio.sleep(0.00021)
    return payload


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
            await asyncio.sleep(0.0001)
        self.crc = accum
        return self

    async def accumulate_str(self, buf: [str, bytes]):
        """
        Add in rolling byte-chunks.
        """
        try:
            bytes_array = array.array('B', list(buf))
        except TypeError:
            bytes_array = array.array('B', list(buf.encode()))
        except AttributeError:
            raise ValueError
        await self.accumulate(bytes_array)
        return self


x25 = X25crc()


async def crc_check(pack: dict, debug: bool = False) -> bool:
    """
    CRC Checking callback.
    """
    result = False
    chk = pack['chk']
    crc = pack['crc']
    try:
        await x25.create(chk)
        crc_extra = formats[pack['message_id']][1]
        await x25.accumulate_str(struct.pack("B", crc_extra))  # noqa
        _crc = x25.crc
        if debug:
            print(crc, _crc)
        if crc == _crc:
            result = True
        if debug and not result:
            print('CRC check failed')
        elif debug:
            print('CRC check passed')
    except KeyError:
        if debug:
            print('Skipping CRC for unincluded message_id', pack['message_id'])
    return result


class Packet:
    """
    Construct/deconstruct Mavlink packets.
    """
    packet = None

    magic = 0xfd  # Start.
    p_len = 0x00  # 0-255  Payload length.
    incompat = 0x00  # 0-255 Incompatibility Flags: https://mavlink.io/en/guide/serialization.html#incompat_flags
    compat = 0x00  # 0-255 Compatibility Flags: https://mavlink.io/en/guide/serialization.html#compat_flags
    psn = 0  # 0-255 Packet sequence number.
    sid = 0x01  # 1-255 System ID (sender).
    cid = 0x01  # 1-255 Component ID (sender): https://mavlink.io/en/messages/common.html#MAV_COMPONENT
    mid = 0x00  # 0-16777215 Message ID (low, middle, high bytes)
    payload: bytes = bytes()  # Payload Max 255 bytes: https://mavlink.io/en/guide/serialization.html#payload
    next_payload: bytes = bytes()  # A place to store payload information that is larger than the allowed size.
    crc = 0xffff  # Checksum (low byte, high byte): https://mavlink.io/en/guide/serialization.html#checksum
    sig = 0x00  # Optional signature: https://mavlink.io/en/guide/message_signing.html

    crc_extra = 0x00

    header = bytes()

    x25 = X25crc()

    def __init__(self):
        self.dummy = None

    async def reset(self):
        """
        Clear all variables to save memory.
        Also increment the PSN.
        """
        self.p_len = 0x00
        self.incompat = 0x00
        self.compat = 0x00
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

        self.psn += 1
        if self.psn > 255:
            self.psn = 0
        return self

    async def create_header(self):
        """
        Constructs a mavlink 2 header.
        """
        self.header = struct.pack(
            "<BBBBBBBHB",  # noqa
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
            await asyncio.sleep(0.0001)
        self.payload = self.payload[:self.p_len]
        return self

    async def assemble(self):
        """
        Assembles a packet for transmission.
        """
        await self.create_payload(self.mid, self.payload)
        await self.create_header()
        self.packet = bytes(self.header) + bytes(self.payload)
        await self.x25.create(self.packet[1:])
        self.crc_extra = formats[self.mid][1]
        await self.x25.accumulate_str(struct.pack("B", self.crc_extra))  # noqa
        self.crc = self.x25.crc
        self.packet += struct.pack("<H", self.crc)
        return self

    async def create_payload(self, message_id, payload):
        """
        Pack the user payload according to the specified format.
        """
        _format = formats[message_id][0]
        if len(payload) != len(_format):
            print('payload malformed: expected', len(_format), 'arguments, received:', len(payload))
            raise ValueError
        _format = "<" + formats[message_id][0]
        # Decode Order
        if message_id <= 255:
            tmpPayload = payload[:]
            for i in range(0, len(payload)):
                payload[formats[message_id][2][i]] = tmpPayload[i]
                await asyncio.sleep(0.00017)
        try:
            self.payload = struct.pack(_format, *payload)
        except OverflowError as err:
            print('Unable to pack payload\n', _format, payload)
            raise err
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
    async def wait(debug: bool = False) -> True:
        """
        Waits for a heartbeat.
        """
        beat = False
        hold = False
        now = monotonic()
        timeout = now + 1
        while not beat and not TERM:
            for idx, message in enumerate(read_buffer):
                if message['message_id'] == 0:
                    hold = message['increment']
                    del read_buffer[idx]
                    if debug:
                        print('\n\n\n\nfound heartbeat **************************************************************************\n\n\n\n')
                await asyncio.sleep(0.00013)
            if hold:
                beat = hold
            if timeout < now:
                print('\n\n\n\nwarning heartbeat timed out **************************************************************************\n\n\n\n')
                break
            await asyncio.sleep(0.0013)
        return beat


class Command:
    """
    This will allow us to send commands and read the ACK.
    """

    packet = Packet()

    def __init__(self, s_id: int, c_id: int):
        self.s_id = s_id
        self.c_id = c_id

    @staticmethod
    async def wait(cmd_id: int, debug: bool = False):
        """
        This will wait for our command ACK response.
        """
        global read_buffer
        ack = False
        now = monotonic()
        timeout = now + 1
        while not ack and not TERM:
            for idx, message in enumerate(read_buffer):
                if message['message_id'] == 77:
                    if message['payload'] == [cmd_id]:
                        del read_buffer[idx]
                        if debug:
                            print('\n\n\n\nfound ACK **************************************************************************\n\n\n\n')
                        break
                await asyncio.sleep(0.00016)
            if timeout < now:
                if debug:
                    print('\n\n\n\nwarning ACK timed out **************************************************************************\n\n\n\n')
            await asyncio.sleep(0.0011)
        return ack


class MavLink:
    """
    This is where it all comes together babah.

    NOTE: When using callbacks for incoming command executions ensure they accept 7 arguments for the params:

    callbacks = {
    246: shutdown_command_function,
    <id>: <command>
    }

    On reception the command parser will:

    ack_payload = shutdown_command_function(command_parameters_seven_long)

    send_ack(payload)

    ack_payload body:

    [<result>, <progress>, <result_param2>]

    """
    global TERM
    term = TERM

    ack = None

    confirmation = 0

    def __init__(
            self,
            message_ids: list,
            s_id: int = defs['sid'],
            c_id: int = defs['cid'],
            c_flags: int = defs['cfl'],
            i_flags: int = defs['ifl'],
            retries: int = 10,
            callbacks: dict = dict(),  # noqa
            heartbeat_payload: list = [18, 8, 0, 0, 0, 3]  # noqa
    ):
        self.heartbeat_payload = heartbeat_payload
        self.system_id = s_id
        self.component_id = c_id
        self.compatibility_flags = c_flags
        self.incompatibility_flags = i_flags
        if callbacks:  # Add any definitions for incoming commands.
            command_ids = list(callbacks.keys())
            message_ids.extend(command_ids)
        self.callbacks = callbacks
        message_ids.extend([76, 77, 0, 111])
        for f in formats:
            if f not in message_ids:
                del formats[f]
        self.message_ids = message_ids
        self.heartbeat = Heartbeat()
        self.packet = Packet()
        self.command = Command(s_id, c_id)

        self.retries = retries

    async def command_parser(self, debug: bool = False):
        """
        This will collect incoming commands and fire off their respective callbacks.
        """
        global read_buffer
        for idx, p in enumerate(read_buffer):
            if p['message_id'] == 76:  # Check if the message is a command.
                if [p['system_id'], p['component_id']] != [self.system_id, self.component_id]:  # Ignore our own commands.
                    pl = await decode_payload(76, p['payload'], debug=debug)
                    sid = pl[0]  # Target system.
                    cid = pl[1]  # Target component.
                    if sid in [0, 255, self.system_id]:
                        if cid in [0, 255, self.component_id]:
                            cmd = pl[2]  # Target command.
                            args = pl[4:]  # Get the seven arguments.
                            if cmd in self.callbacks.keys():

                                prefix = [cmd]
                                suffix = [p['system_id'], p['component_id']]

                                callback = self.callbacks[cmd]
                                post = False
                                if cmd == 246:  # Determine if we have been commanded to shut down.
                                    post = True
                                    ack_payload = prefix + [0, 0, 0] + suffix
                                else:
                                    ack_payload = await callback(*args)
                                    if len(ack_payload) != 3:
                                        print('ERROR: callback results should have exactly three members, not', len(ack_payload))
                                        raise ValueError
                                    ack_payload = prefix + ack_payload + suffix
                                await self.send_message(  # Send command ACK.
                                    77,
                                    ack_payload,
                                    s_id=self.system_id,
                                    c_id=self.component_id
                                )
                                if post:  # If the command is shutdown/restart, execute after the ack has been sent.
                                    await callback(*args)
                            elif debug:
                                print('received command', cmd, 'without format definition')
                            del read_buffer[idx]
            await asyncio.sleep(0.00018)

    async def heartbeat_wait(self, debug: bool = False):
        """
        Wait for heartbeat packet.
        """
        result = await self.heartbeat.wait(debug)  # noqa
        return result

    async def ack_wait(self, command_id: int, debug: bool = False):
        """
        This will wait for a command ACK up to a specific timeout.
        """
        result = await self.command.wait(command_id, debug)  # noqa
        return result

    async def send_message(
            self,
            message_id: int,
            payload: [list, tuple],
            s_id: int = defs['sid'],
            c_id: int = defs['cid'],
            c_flags: int = defs['cfl'],
            i_flags: int = defs['ifl'],
            debug: bool = False  # noqa
    ) -> Packet:
        """
        Formats an outgoing message into a Mavlink packet.
        """
        try:
            await self.packet.create_packet(
                message_id,
                payload,
                c_flags,
                i_flags,
                s_id,
                c_id
            )
            return await self.packet.send()
        except KeyError as err:
            print(err, '\nUnable to locate command ID, please check includes and definitions')

    async def send_command(
            self,
            command_id: int,
            target_system: int = 0x01,
            target_component: int = 0x01,
            params: (int, int, int, int, int, int, int) = (0, 0, 0, 0, 0, 0, 0),
            s_id: int = defs['sid'],
            c_id: int = defs['cid'],
            c_flags: int = defs['cfl'],
            i_flags: int = defs['ifl'],
            debug: bool = False
    ) -> Packet:
        """
        Send a command with a 7 byte payload and wait for ACK.
        """
        payload = [target_system, target_component, command_id, self.confirmation]
        payload.extend(list(params))
        await self.packet.create_packet(
            76,
            payload,
            c_flags=c_flags,
            i_flags=i_flags,
            s_id=s_id,
            c_id=c_id
        )
        result = await self.packet.send()
        if target_system != 255:  # Skip ACK wait for broadcast messages.
            self.ack = await self.ack_wait(command_id, debug)  # noqa
            if not self.ack and self.retries:
                self.retries -= 1
                self.confirmation += 1
                await self.send_command(
                    command_id,
                    target_system,
                    target_component,
                    params,
                    c_flags,
                    i_flags,
                    s_id,
                    c_id,
                    debug
                )
                if debug:
                    print('ACK timeout, retrying command', self.retries)

            self.retries = 10
            self.confirmation = 0
            self.ack = None
        return result

    async def receive(self):
        """
        Read from the incoming buffer... We need to break this out into receive_message and receive_command.

        decode COMMAND_ACK: struct.unpack("<HBBiBB", bytes(payload))

        | command | result | progress | result_param2 | target_system | target_component |

        https://mavlink.io/en/messages/common.html#MAV_RESULT
        """
        return await self.packet.receive()

    async def io_buffers(self, uart: any, debug: bool = False):
        """
        This will return the tasks required to handle UART I/O via our read and write buffers.
        """
        parent = self
        _uart = uart
        _debug = debug

        async def read_loop():
            """
            Read buffer loop.
            """
            global read_buffer
            global TERM
            while not TERM:
                read_buffer = await uart_read(_uart, crc_check, _debug)
                gc.collect()

        async def write_loop():
            """
            Write loop.
            """
            global TERM
            while not TERM:
                await uart_write(_uart, _debug)

        async def heartbeat_loop():
            """
            Let's send some heartbeat packets.
            """
            global TERM
            while not TERM:
                await parent.send_message(0, self.heartbeat_payload, c_flags=0, i_flags=0, s_id=self.system_id, c_id=self.component_id)
                await asyncio.sleep(1)

        async def command_loop():
            """
            Listen for incoming commands.
            """
            global TERM
            while not TERM:
                await parent.command_parser(_debug)

        result = [
            write_loop(),
            read_loop(),
            heartbeat_loop(),
            command_loop()
        ]
        return result

    def __exit__(self, *args):
        """
        Bail out of the runtime.
        """
        global TERM
        TERM = False
        self.term = True


async def test(_uart):
    """
    This is a simple test method that sends and receives restart commands.
    """
    async def reboot(*args):  # noqa
        """
        This is a simple example of an incoming command callback function.
        """
        # A real command should evaluate the 7 bytes of param data that will be passed as args.
        import microcontroller  # noqa
        microcontroller.reset()  # noqa
        # Any other command would need to return status information.
        # return 0, 0, 0

    m_id = 246
    m = MavLink([m_id, 253], callbacks={246: reboot})

    async def send():
        """Send test command"""
        await m.heartbeat_wait(True)
        await m.send_command(
            command_id=246,
            target_system=0,
            target_component=0,
            params=[1, 0, 0, 0, 0, 0, 0],
            debug=True
        )
        print('\n\n\n\n----------------------------- IF YOU SEE THIS THERE HAS BEEN AN ERROR -----------------------------------\n\n\n\n')

    async def main(uart_):
        """
        Test mainloop.
        """
        tasks = await m.io_buffers(uart_, debug=True)
        snd_cmd = asyncio.create_task(send())
        await asyncio.gather(snd_cmd, *tasks)

    asyncio.run(main(_uart))
