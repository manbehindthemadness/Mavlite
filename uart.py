# Write your code here :-)
import board


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
            timeout: int = 1,
            rxbuf: int = 64,
            tx: any = None,
            rx: any = None
    ):
        if self.mp:
            self.uart = self.uart(s_id, baudrate)
            self.uart.init(baudrate, bits, parity, stop, timeout=timeout, rxbuf=rxbuf, rx=rx, tx=tx)
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

    def read(self, n_bytes: int = 8) -> uart.read:
        """
        Read wrapper.
        """
        return self.uart.read(n_bytes)

    def readinto(self, buf: bytes) -> uart.readinto:
        """
        Readinto wrapper.
        """
        return self.readinto(buf)

    def readline(self) -> uart.readline:
        """
        Readline wrapper.
        """
        return self.uart.readline()

    def write(self, buf: [str, bytes]) -> uart.write:
        """
        Write wrapper
        """
        if not isinstance(buf, bytes):
            buf = buf.encode()
        return self.uart.write(buf)

    def deinit(self) -> uart.deinit:
        """
        Deinit wrapper.
        """
        return self.uart.deinit()

    def setbaudrate(self, baudrate: int):
        """
        Set baudrate wrapper.
        """
        if self.mp:
            return self.uart.setbaudrate(baudrate)
        else:
            self.uart.baudrate = baudrate
            return self.uart

    def any(self):
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


uart = UART(tx=board.TX, rx=board.RX, baudrate=115200)

stream = list()
packets = list()


# async def receive():
while True:
    """
    Uart packet listener.
    """
    # global stream
    # global packets
    partial_buff = list()
    raw = uart.read(64)
    data = list(raw)  # read up to 64 bytes
    if data is not None:
        for idx, byte in enumerate(range(len(data))):
            if data[byte] == 0xfd:  # Check for start condition.
                if stream:  # If we have a large partial stored from the last read iteration, finish it.
                    stream.extend(partial_buff)
                    packets.append(stream)
                elif partial_buff:  # If the packets are smaller than the read buffer, handle them independently.
                    packets.append(partial_buff)
                partial_buff = list()
                stream = list()
            partial_buff.append(data[byte])
        stream.extend(partial_buff)
        if len(packets):
            for p in packets:
                print('--------------------')
                try:
                    pay_end = 10 + p[1]
                    msg = f'start {p[0]}, length {p[1]}, incompat {p[2]}, compat {p[3]}, seq {p[4]}, sys_id {p[5]}, '
                    msg += f'comp_id {p[6]}, mes_id {p[7:9]}, payload {p[10:pay_end]}, chk {p[pay_end:pay_end + 2]}'
                except IndexError:
                    msg = f'bad_data {p}'
                print(msg)
            packets = list()
