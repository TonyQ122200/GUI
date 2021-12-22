from queue import Queue
import threading
import serial
import struct
import math


class Helmet:
    __samples_per_second = 1000
    __accelerometer_range = 400     # range of accelerometer is +-200g
    __gyroscope_range = 4000        # range of gyroscope is +-2000dps

    __timebase = 0
    __previous_gyro = []
    __G_CONVERSION_FACTOR = __accelerometer_range / 65536
    __RADS_CONVERSION_FACTOR = (__gyroscope_range * math.pi * __samples_per_second) / (65536 * 180)

    __CONCUSS_ACC_LOWER = 35
    __CONCUSS_ACC_UPPER = 102.6
    __CONCUSS_GYRO_LOWER = 2000
    __CONCUSS_GYRO_UPPER = 8445.6
    concussion_flag = False
    __big_impacts = []

    graph_data = False
    graph_q = Queue(maxsize=0)

    def __init__(self, com_port, name):
        self.__com_port = com_port
        self.name = name

    def start(self):
        processing_q = Queue(maxsize=0)       # setting maxsize to zero makes queue infinitely long
        with serial.Serial(self.__com_port, baudrate=115200, timeout=2) as ser:
            ser_thread = threading.Thread(target=self.__receive_data, args=(ser, processing_q))
            ser_thread.start()
            while True:
                data = processing_q.get(True, 3)    # blocking for 3 seconds
                # process data
                if self.__threshold(data):
                    self.concussion_flag = True
                    self.__big_impacts.append(data)
                    # cloud(data)
                    print("threshold")
                    print(data)

    def __receive_data(self, ser, processing_q):
        while True:
            id_byte = ser.read()
            # check byte
            if id_byte == b'A':
                raw_values = struct.unpack('<hhhhhh', ser.read(size=12))

                values = self.__decode_values(raw_values)

                # create an event with a time and acceleration values
                values.insert(0, self.__timebase)
                event = tuple(values)

                processing_q.put(event)
                self.__timebase += 1

    def __decode_values(self, raw_values):
        decoded_list = []  # to be filled with acceleration values

        # calculate accelerometer values in g
        for i in range(3):
            acc_val = raw_values[i]
            val_g = acc_val*self.__G_CONVERSION_FACTOR
            decoded_list.append(val_g)

        # calculate gyroscope values in rads^-2
        for j in range(3, 6):
            gyro_val = raw_values[j]

            if self.__timebase == 0:
                # there are no previous values therefore let initial acceleration be 0
                self.__previous_gyro.append(gyro_val)

            gyro_diff = gyro_val - self.__previous_gyro[j-3]    # calculating the change in gyroscope value
            val_rads = gyro_diff * self.__RADS_CONVERSION_FACTOR  # calculating acceleration in (rads^-2)

            decoded_list.append(val_rads)
            self.__previous_gyro[j-3] = gyro_val

        return decoded_list

    def __threshold(self, data_tuple):
        accelerometer = False
        gyroscope = False
        concussion = False

        for i in range(1, 4):
            if self.__CONCUSS_ACC_LOWER < abs(data_tuple[i]) < self.__CONCUSS_ACC_UPPER:
                accelerometer = True

        for j in range(4, 7):
            if self.__CONCUSS_GYRO_LOWER < abs(data_tuple[j]) < self.__CONCUSS_GYRO_UPPER:
                gyroscope = True

        if accelerometer and gyroscope:
            concussion = True

        return concussion

    """
    def cloud(big_impact):
        return 1
    """


if __name__ == '__main__':
    helmet4 = Helmet('COM4', 'Gerald')
    helmet4.start()
