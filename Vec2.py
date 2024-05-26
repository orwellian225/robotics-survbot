import math as m

class Vec2():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def magnitude(self):
        return m.sqrt(self.x**2 + self.y**2)

    def distance_to(self, other):
        return (self - other).magnitude()

    def angle_to(self, other):
        return m.acos( self.dot(other) / ( self.magnitude() * other.magnitude() ) )
