# Write your code here :-)
# SPDX-FileCopyrightText: 2018 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""CircuitPython Essentials UART Serial example"""
import board
import busio
import digitalio

uart = busio.UART(board.TX, board.RX, baudrate=115200)

stream = list()
packets = list()


def find_with_list(myList, target):
     inds = []
     for i in range(len(myList)):
         if myList[i] == target:
             inds += i,
     return inds

while True:
    try:
        partial_buff = list()
        raw = uart.read(64)
        # print(raw)
        data = list(raw)  # read up to 64 bytes

        if data is not None:
            beginning = False
            for idx, byte in enumerate(range(len(data))):
                if data[byte] == 0xfd:  # Check for start condition.
                    # print('found start', len(stream), len(partial_buff))
                    if stream:  # If we have a large partial stored from the last read iteration, finish it.
                        # print('finishing large partial')
                        stream.extend(partial_buff)
                        packets.append(stream)
                    elif partial_buff:  # If the packets are smaller than the read buffer, handle them independently.
                        packets.append(partial_buff)
                        # print('finishing small partial')
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
    except KeyboardInterrupt:
        pass
