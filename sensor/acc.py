from sensor.sensor_axis import SensorAxis
class Acc(SensorAxis):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x, y, z)
        
        
    def set_abs(self):
        # self.x = abs(self.x)
        # self.y = abs(self.y)
        # self.z = abs(self.z)
        
        self.x *= 1
        self.y *= 9.8
        self.z *= 1
        
        
# 23
# 17
# 14
# 8
# 3
# 39