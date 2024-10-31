from sensor.sensor_axis import SensorAxis


class Mag(SensorAxis):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x, y, z)
