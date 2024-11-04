from sensor.sensor_axis import SensorAxis
import math


class Acc(SensorAxis):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x, y, z)
        
        
    def set_abs(self):
        # self.x = abs(self.x)
        # self.y = abs(self.y)
        # self.z = abs(self.z)
        
        self.x *= 0.981
        self.y *= 0.981
        self.z *= 0.981
        
        
    def norm(self):
        if self.x == 0 or self.y == 0 or self.z == 0:
            return 0
        
        
        n_data = math.sqrt(self.x**2 + self.y**2 + self.z**2)
        self.x /= n_data
        self.y /= n_data
        self.z /= n_data
        
        
# 23
# 17
# 14
# 8
# 3
# 39