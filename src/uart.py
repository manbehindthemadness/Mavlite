"""# Write your code here :-)"""
import struct
from math import sqrt
# import re
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
variance = list()
assignments = list()
VALID_SENDERS = [
    (1, 1),
]
VALID_MESSAGES = list()


class Equalizer:
    """
    This will issue unique task-delay values in the form of prime numbers.
    """
    def __init__(self, spread: int = 100):
        global variance
        global assignments
        self.spread = spread
        self.get_primes()

    @staticmethod
    def isprime(n):
        """
        Determines if a candidate is a prime number.
        """
        prime_flag = 0

        if n > 1:
            for i in range(2, int(sqrt(n)) + 1):
                if n % i == 0:
                    prime_flag = 1
                    break
            if prime_flag == 0:
                return True
            else:
                return False
        else:
            return False

    def get_primes(self):
        """
        Gets a unique prime delay times.
        """
        global variance
        global assignments
        for number in range(self.spread):
            if self.isprime(number) and number not in variance:
                variance.append(number / 100000)
        if not assignments:
            assignments = [0] * len(variance)
        else:
            variance_length = len(variance)
            assignments.extend([0] * (len(assignments) - variance_length))
        return self

    @staticmethod
    def assign(delay_var: [int, float] = 0, base: [int, float] = 0):
        """
        This will assign a unique delay time.
        """
        if not delay_var:
            global assignments
            result = None
            for idx, (assignment, variant) in enumerate(zip(assignments, variance)):
                if not assignment:
                    result = variant
                    assignments[idx] = 1
                    break
            if result is None:
                print('Out of assignments, please increase variance spread')
                print(variance)
                print(assignments)
                raise RuntimeError
            result += base
            delay_var = result
        return delay_var

    async def wait(self, delay_var: [int, float] = 0, base: [float, int] = 0):
        """
        Returns the wait action.
        """
        delay_var = self.assign(delay_var, base)
        await asyncio.sleep(delay_var)
        return delay_var


waits = Equalizer()


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


wait = 0


async def none_wait(callback, *args):
    """
    Waits for the variable condition to not be none and then returns the result
    """
    global VALID_SENDERS
    global VALID_MESSAGES
    global wait
    condition = None

    while condition is None and [len(VALID_MESSAGES), len(VALID_SENDERS)] != [0, 0]:
        condition = await callback(*args)
        wait = await waits.wait(wait)
    return condition


wait_1 = 0


async def uart_read_2(
        _uart: any = None,
        callback: any = None,
        debug: bool = False
):
    """
    A better uart packet listener.
    """
    packet_start, payload_length, packet, message_id, crc = [None] * 5
    global wait_1
    global read_buffer
    global VALID_MESSAGES
    global VALID_SENDERS
    global has_start_point
    if not has_start_point:
        while not has_start_point:  # Find the start point inbetween packets.
            pointer = await none_wait(_uart.read, 1)
            pointer = int(list(pointer)[0])
            # await asyncio.sleep(0.5)
            wait_1 = await waits.wait(wait_1)
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
            except (TypeError, IndexError):
                if debug:
                    print('dropping incomplete packet', packet)
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


wait_2 = wait_3 = 0


async def uart_write(_uart: any, debug: bool = False):
    """
    Writes our buffer to the device
    """
    global wait_2
    global wait_3
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
            del write_buffer[idx]  # No delay here.
    else:
        # await asyncio.sleep(0.023)
        wait_3 = await waits.wait(wait_3, 0.5)  # noqa THIS IS PERFORMANCE CRITICAL.
    return write_buffer


async def uart_io(_uart: any, callback: any = None, debug: bool = False):
    """
    Mainloop for this operation.
    """
    await uart_write(_uart, debug)
    await uart_read_2(_uart, callback, debug)


wait_4 = 0


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
        global wait_4
        _uart = UART(tx=board.TX, rx=board.RX, baudrate=230400)
        while True:
            try:
                await uart_io(_uart, debug=True)
            except KeyboardInterrupt:
                break
            # await asyncio.sleep(0.00014)
            wait_4 = await waits.wait(wait_4)

    asyncio.run(loop())
