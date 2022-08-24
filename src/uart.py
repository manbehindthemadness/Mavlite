"""# Write your code here :-)"""
import struct
try:
    import asyncio
except ImportError:
    import uasyncio as asyncio  # noqa

magic = [253]


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


TERM = False
stream = list()
packets = list()
read_buffer = list()
write_buffer = list()


buffer_size = 8


async def uart_read(_uart: any = None, callback: any = None, debug: bool = False) -> list:
    """
    Uart packet listener.

    TODO: This needs to take into account message includes so we don't spend all this processing time decoding every
            packet.

    """
    global stream
    global packets
    global read_buffer
    partial_buff = list()
    raw = await _uart.read(64)
    if raw:
        data = list(raw)  # read up to 64 bytes
        skip = 0
        if data:
            for idx, byte in enumerate(range(len(data))):
                if data[byte] in magic and not skip:  # Check for start condition.
                    try:  # Don't check for another magic byte until after the end of this packet.
                        p_len = data[idx + 1]
                        min_length = 12 + p_len
                        # TODO: We will need to add 13 bytes here if the communication is signed.
                        skip = min_length
                    except IndexError:
                        pass
                    if stream:  # If we have a large partial stored from the last read iteration, finish it.
                        stream.extend(partial_buff)
                        packets.append(stream)
                    elif partial_buff:  # If the packets are smaller than the read buffer, handle them independently.
                        packets.append(partial_buff)
                    partial_buff = list()
                    stream = list()
                partial_buff.append(data[byte])
                if skip:
                    skip -= 1
                await asyncio.sleep(0.0015)
            stream.extend(partial_buff)
            if len(packets):
                for idx, p in enumerate(packets):
                    try:
                        if 12 <= len(p) <= 280:  # 12 is the minimum MavLink packet length.
                            if debug:
                                print('--------------------')
                            try:
                                message_id = struct.unpack("H", bytes(p[7:9]))[0]
                            except (RuntimeError, ValueError) as err:
                                print('Unable to unpack message_id', err, '\n', p[7:9])
                                del packets[idx]  # Clear memory.
                                break
                            pay_end = 10 + p[1]
                            payload = p[10:pay_end]
                            try:  # Prevent a failure from snowballing into logic down the line.
                                crc = struct.unpack("H", bytes(p[pay_end:pay_end + 2]))[0]
                            except RuntimeError as err:
                                print(err, 'unable to decode crc\n', p, '\n', bytes(p))
                                del packets[idx]  # Clear memory.
                                break  # Prevent a failure from snowballing into logic down the line.
                            chk = p[1:pay_end]

                            msg = f'start {p[0]}, length {p[1]}, incompat {p[2]}, compat {p[3]}, seq {p[4]}, sys_id {p[5]}, '
                            msg += f'comp_id {p[6]}, mes_id {message_id}, '
                            msg += f'payload {payload}, crc {crc}, \nraw {bytes(p)}'
                            pack = {
                                'message_id': message_id,
                                'system_id': p[5],
                                'component_id': p[6],
                                'payload': payload,
                                'increment': p[4],
                                'crc': crc,
                                'chk': chk,
                                'raw': bytes(p)
                            }
                            if callback:  # For CRC checking.
                                if await callback(pack, debug):
                                    read_buffer.append(pack)
                            read_buffer = read_buffer[-buffer_size:]
                        else:
                            msg = f'dropping packet: {p}'
                            del packets[idx]  # Clear memory.
                    except IndexError:
                        msg = f'bad_data {p}'
                        del packets[idx]  # Clear memory.
                    if debug:
                        print('--------------------\n', 'receiving', msg)
                    await asyncio.sleep(0.002)
                packets = list()
    else:
        await asyncio.sleep(0.001)
    return read_buffer


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
    await uart_read(_uart, callback, debug)


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
        _uart = UART(tx=board.TX, rx=board.RX, baudrate=115200)
        while True:
            try:
                await uart_io(_uart, True)
            except KeyboardInterrupt:
                break
            await asyncio.sleep(0.00014)

    asyncio.run(loop())
