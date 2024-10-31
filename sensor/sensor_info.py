from sensor.acc import Acc
from sensor.gyro import Gyro
from sensor.mag import Mag
from sensor.quaternion import Quaternion


class SensorInfo():
    def __init__(self, part, acc=Acc(), gyro=Gyro(), mag=Mag(), quaternion=Quaternion()):
        self.__part = part
        self.__acc = acc
        self.__gyro = gyro
        self.__mag = mag
        self.__quaternion = quaternion

    @property
    def part(self):
        return self.__part

    @part.setter
    def part(self, value):
        self.__part = value

    @property
    def acc(self):
        return self.__acc

    @acc.setter
    def acc(self, value):
        self.__acc = value

    @property
    def gyro(self):
        return self.__gyro

    @gyro.setter
    def gyro(self, value):
        self.__gyro = value

    @property
    def mag(self):
        return self.__mag

    @mag.setter
    def mag(self, value):
        self.__mag = value

    @property
    def quaternion(self):
        return self.__quaternion

    @quaternion.setter
    def quaternion(self, value):
        self.__quaternion = value


