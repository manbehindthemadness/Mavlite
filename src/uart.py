"""# Write your code here :-)"""
import struct
try:
    import asyncio
except ImportError:
    import uasyncio as asyncio  # noqa

magic = [253]
TERM = False
stream = list()
packets = list()
read_buffer = list()
write_buffer = list()


buffer_size = 8


VALID_SENDERS = [
    (1, 1),
]
VALID_MESSAGES = [
    0,
    76,
    77
]


class UART:
    # noinspection GrazieInspection
    """
        Wrapper class for machine.UART and busio.UART
        """
    try:
        mp = True
        from machine import UART as uart  # noqa
    except ImportError:
        mp = False
        from busio import UART as uart  # noqa

    def __init__(
            self,
            s_id: int = 1,
            baudrate: int = 57600,
            bits: int = 8,
            parity: [None, int] = None,
            stop: int = 1,
            timeout: int = 0,
            rxbuf: int = 64,
            tx: any = None,
            rx: any = None
    ):
        if self.mp:
            self.uart = self.uart(s_id, baudrate)
            self.uart.init(baudrate, bits=bits, parity=parity, stop=stop, timeout=timeout, tx=tx, rx=rx)
        else:
            self.uart = self.uart(
                tx, rx,
                baudrate=baudrate,
                bits=bits,
                parity=parity,
                stop=stop,
                timeout=timeout,
                receiver_buffer_size=rxbuf
            )

    async def read(self, n_bytes: int = 8) -> uart.read:
        """
        Read wrapper.
        """
        if self.mp:
            result = None
            Timeout = 3000
            while result is None and Timeout:
                result = self.uart.read(n_bytes)
                Timeout += 1
                await asyncio.sleep(0.0002)
            return result
        else:
            return self.uart.read(n_bytes)

    async def readinto(self, buf: bytes) -> uart.readinto:
        """
        Readinto wrapper.
        """
        return self.readinto(buf)

    async def readline(self) -> uart.readline:
        """
        Readline wrapper.
        """
        return self.uart.readline()

    async def write(self, buf: [str, bytes]) -> uart.write:
        """
        Write wrapper
        """
        if not isinstance(buf, bytes):
            buf = buf.encode()
        return self.uart.write(buf)

    async def deinit(self) -> uart.deinit:
        """
        Deinit wrapper.
        """
        return self.uart.deinit()

    async def setbaudrate(self, baudrate: int):
        """
        Set baudrate wrapper.
        """
        if self.mp:
            return self.uart.setbaudrate(baudrate)
        else:
            self.uart.baudrate = baudrate
            return self.uart

    async def any(self):
        """
        Any wrapper.
        """
        if self.mp:
            result = self.uart.any()
        else:
            result = self.uart.in_waiting
        if result:
            print('bytes waiting', result)
        return result


def messages(valid_messages: [list, None] = None):
    """
    Allows to drop unneeded message packets to save cycles.
    """
    global VALID_MESSAGES
    VALID_MESSAGES = list()
    if valid_messages:
        VALID_MESSAGES = valid_messages
    return VALID_MESSAGES


def check_message(message_id: int) -> bool:
    """
    Cecks to see if we should keep the message, or drop it to save memory.
    """
    global VALID_MESSAGES
    return message_id in VALID_MESSAGES


def senders(valid_senders: [list, None] = None):
    """
    Allows us to set the criteria for messages we will include in the read buffer.

    The idea here is to allow conditional packet reception so, we don't bother decoding packets that aren't relevant.
    """
    global VALID_SENDERS
    VALID_SENDERS = list()
    if valid_senders:
        VALID_SENDERS = valid_senders
    return VALID_SENDERS


def check_sender(packet: list, debug: bool = False) -> bool:
    """
    Used to determine if we should drop a packet.
    """
    global VALID_SENDERS
    try:
        sender = packet[5], packet[6]
        return sender in VALID_SENDERS
    except IndexError:
        if debug:
            print('unable to check sender on partial packet')
        return False


has_start_point = False


async def none_wait(callback, *args):
    """
    Waits for the variable condition to not be none and then returns the result
    """
    condition = None
    while condition is None:
        condition = await callback(*args)
        await asyncio.sleep(0.0015)
    return condition


async def uart_read_2(
        _uart: any = None,
        callback: any = None,
        debug: bool = False
):
    """
    A better uart packet listener.
    """
    packet_start, payload_length, packet, message_id, crc = [None] * 5
    global read_buffer
    global VALID_MESSAGES
    global VALID_SENDERS
    global has_start_point
    if not has_start_point:
        while not has_start_point:  # Find the start point inbetween packets.
            pointer = await none_wait(_uart.read, 1)
            pointer = int(list(pointer)[0])
            await asyncio.sleep(0.5)
            if pointer == 0xFD:
                has_start_point = True
                payload_length = await _uart.read(1)
                payload_length = int(list(payload_length)[0])
                packet_start = [pointer] + [payload_length]
                if debug:
                    print('found start point')
    else:
        packet_start = await _uart.read(2)
        try:
            packet_start = list(packet_start)
            payload_length = int(packet_start[1])
        except (TypeError, IndexError):
            has_start_point = False
    if has_start_point:
        fault = False
        remaining_packet_length = payload_length + 10
        remaining_packet_content = await _uart.read(remaining_packet_length)
        try:
            packet = packet_start + list(remaining_packet_content)
            message_id = struct.unpack("H", bytes(packet[7:9]))[0]
        except (RuntimeError, TypeError) as err:
            if debug:
                print('dropping packet with malformed message ID', err)
                fault = True
        if not fault:
            excluded = True
            try:
                if (packet[5], packet[6]) in VALID_SENDERS and message_id in VALID_MESSAGES:
                    excluded = False
            except TypeError:
                print('PACKET', packet)
                raise TypeError
            if not excluded:
                pay_end = 10 + packet[1]
                payload = packet[10:pay_end]
                try:
                    crc = struct.unpack("H", bytes(packet[pay_end:pay_end + 2]))[0]
                except RuntimeError as err:
                    if debug:
                        print('dropping packet with malformed payload', err)
                        fault = True
                if not fault:
                    chk = packet[1:pay_end]
                    msg = f'start {packet[0]}, length {packet[1]}, incompat {packet[2]}, compat {packet[3]}, seq {packet[4]}, sys_id {packet[5]}, '
                    msg += f'comp_id {packet[6]}, mes_id {message_id}, '
                    msg += f'payload {payload}, crc {crc}, \nraw {bytes(packet)}'
                    pack = {
                        'message_id': message_id,
                        'system_id': packet[5],
                        'component_id': packet[6],
                        'payload': payload,
                        'increment': packet[4],
                        'crc': crc,
                        'chk': chk,
                        'raw': bytes(packet)
                    }
                    if callback:  # For CRC checking.
                        if await callback(pack, debug):
                            read_buffer.append(pack)
                    read_buffer = read_buffer[-buffer_size:]
                    if debug:
                        print(msg)
    return read_buffer


# async def uart_read(
#         _uart: any = None,
#         callback: any = None,
#         debug: bool = False
# ) -> list:
#     """
#     Uart packet listener.
#
#     TODO: This needs to be enhanced to use the packet length byte to read variable lengths of whole packets from the
#             RX buffer instead of constantly iterating through every single byte.
#     """
#     global stream
#     global packets
#     global read_buffer
#
#     partial_buff = list()
#     raw = await _uart.read(64)
#     if raw:
#         data = list(raw)  # read up to 64 bytes
#         skip = 0
#         if data:
#             for idx, byte in enumerate(range(len(data))):
#                 if data[byte] == 253 and not skip:  # Check for start condition.
#                     try:  # Don't check for another magic byte until after the end of this packet.
#                         p_len = data[idx + 1]
#                         min_length = 12 + p_len
#                         # TODO: We will need to add 13 bytes here if the communication is signed.
#                         skip = min_length
#                     except IndexError:
#                         pass
#                     if stream:  # If we have a large partial stored from the last read iteration, finish it.
#                         stream.extend(partial_buff)
#                         if check_sender(stream, debug):
#                             packets.append(stream)
#                         elif debug:
#                             print('skipping packet', stream)
#                     elif partial_buff:  # If the packets are smaller than the read buffer, handle them independently.
#                         if check_sender(partial_buff, debug):
#                             packets.append(partial_buff)
#                         elif debug:
#                             print('skipping packet', partial_buff)
#                     partial_buff = list()
#                     stream = list()
#                 partial_buff.append(data[byte])
#                 if skip:
#                     skip -= 1
#                 await asyncio.sleep(0.0015)
#             stream.extend(partial_buff)
#             if len(packets):
#                 for idx, p in enumerate(packets):
#                     try:
#                         if 12 <= len(p) <= 280:  # 12 is the minimum MavLink packet length.
#                             if debug:
#                                 print('--------------------')
#                             try:
#                                 message_id = struct.unpack("H", bytes(p[7:9]))[0]
#                                 if not check_message(message_id):
#                                     if debug:
#                                         print('dropping excluded message', p)
#                                     del packets[idx]  # Clear memory.
#                                     break
#                             except (RuntimeError, ValueError) as err:
#                                 print('Unable to unpack message_id', err, '\n', p[7:9])
#                                 del packets[idx]  # Clear memory.
#                                 break
#                             pay_end = 10 + p[1]
#                             payload = p[10:pay_end]
#                             try:  # Prevent a failure from snowballing into logic down the line.
#                                 crc = struct.unpack("H", bytes(p[pay_end:pay_end + 2]))[0]
#                             except RuntimeError as err:
#                                 print(err, 'unable to decode crc\n', p, '\n', bytes(p))
#                                 del packets[idx]  # Clear memory.
#                                 break  # Prevent a failure from snowballing into logic down the line.
#                             chk = p[1:pay_end]
#
#                             msg = f'start {p[0]}, length {p[1]}, incompat {p[2]}, compat {p[3]}, seq {p[4]}, sys_id {p[5]}, '
#                             msg += f'comp_id {p[6]}, mes_id {message_id}, '
#                             msg += f'payload {payload}, crc {crc}, \nraw {bytes(p)}'
#                             pack = {
#                                 'message_id': message_id,
#                                 'system_id': p[5],
#                                 'component_id': p[6],
#                                 'payload': payload,
#                                 'increment': p[4],
#                                 'crc': crc,
#                                 'chk': chk,
#                                 'raw': bytes(p)
#                             }
#                             if callback:  # For CRC checking.
#                                 if await callback(pack, debug):
#                                     read_buffer.append(pack)
#                             read_buffer = read_buffer[-buffer_size:]
#                         else:
#                             msg = f'dropping packet: {p}'
#                             del packets[idx]  # Clear memory.
#                     except IndexError:
#                         msg = f'bad_data {p}'
#                         del packets[idx]  # Clear memory.
#                     if debug:
#                         print('--------------------\n', 'receiving', msg)
#                     await asyncio.sleep(0.002)
#                 packets = list()
#     else:
#         await asyncio.sleep(0.001)
#     return read_buffer


async def uart_write(_uart: any, debug: bool = False):
    """
    Writes our buffer to the device
    """
    global write_buffer
    if len(write_buffer):
        for idx, packet in enumerate(write_buffer):
            if debug:
                p = list(packet)
                pay_end = 10 + p[1]
                msg = f'start {p[0]}, length {p[1]}, incompat {p[2]}, compat {p[3]}, seq {p[4]}, sys_id {p[5]}, '
                msg += f'comp_id {p[6]}, mes_id {struct.unpack("H", bytes(p[7:9]))[0]}, '
                msg += f'payload {p[10:pay_end]}, crc {struct.unpack("H", bytes(p[pay_end:pay_end + 2]))[0]}'
                msg += f'\nraw {bytes(p)}'
                print('--------------------\n', 'sending', msg)
            await _uart.write(bytes(packet))
            del write_buffer[idx]
            await asyncio.sleep(0.0017)
    else:
        await asyncio.sleep(0.023)
    return write_buffer


async def uart_io(_uart: any, callback: any = None, debug: bool = False):
    """
    Mainloop for this operation.
    """
    await uart_write(_uart, debug)
    await uart_read_2(_uart, callback, debug)


def test():
    """
    Run a test loop
    """
    import board  # noqa
    import asyncio

    async def loop():
        """
        A while clause...
        """
        _uart = UART(tx=board.TX, rx=board.RX, baudrate=230400)
        while True:
            try:
                await uart_io(_uart, debug=True)
            except KeyboardInterrupt:
                break
            await asyncio.sleep(0.00014)

    asyncio.run(loop())
