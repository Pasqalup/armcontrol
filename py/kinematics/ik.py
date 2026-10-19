import math



class IK:
    def __init__(self, l1, l2):
        self.l1 = l1
        self.l2 = l2
    def solve(self, x, y, z):
        # calculate swivel base angle 
        base_angle = math.atan2(y, x)
        # project the target point onto the plane of the arm
        projected_x = math.sqrt(x**2 + y**2)
        projected_y = z
        # calculate the distance from the base to the target point
        distance = math.sqrt(projected_x**2 + projected_y**2)
        # check if the target point is reachable
        if distance > self.l1 + self.l2:
            raise ValueError("Target point is out of reach")
        # calculate the angle of the first joint using the law of cosines
        cos_angle1 = (self.l1**2 + distance**2 - self.l2**2) / (2 * self.l1 * distance)
        angle1 = math.acos(cos_angle1)
        # calculate the angle of the second joint using the law of cosines
        cos_angle2 = (self.l1**2 + self.l2**2 - distance**2) / (2 * self.l1 * self.l2)
        angle2 = math.acos(cos_angle2)
        # calculate the angle of the first joint relative to the horizontal
        joint1_angle = math.atan2(projected_y, projected_x) + angle1
        # calculate the angle of the second joint relative to the first joint
        joint2_angle = math.pi - angle2
        return base_angle, joint1_angle, joint2_angle