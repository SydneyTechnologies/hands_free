# This class should handle everything relating to odometry for a differential drive robot
import math 


class OdomData:
    robot_x = 0
    robot_y = 0
    robot_theta = 0
    robot_linear = 0
    robot_angular = 0

    angular_velocities = []
    angular_positions = []


class Odometry:
    x_ = 0
    y_ = 0
    heading_ = 0
    complimentary_heading = 0

    prev_l_ticks = 0
    prev_r_ticks = 0

    wheel_radius = 0
    wheel_seperation = 0
    ticks_per_rotation = 1


    def __init__(self, wheel_radius, wheel_seperation, ticks_per_rotation):
        self.wheel_radius = wheel_radius
        self.wheel_seperation = wheel_seperation
        self.ticks_per_rotation = ticks_per_rotation


    def updateOdometry(self, l_ticks, r_ticks, dt, theta = 0) -> OdomData:

        l_dist = ((l_ticks - self.prev_l_ticks)/self.ticks_per_rotation) * math.pi * 2 * self.wheel_radius 
        r_dist = ((r_ticks - self.prev_r_ticks)/self.ticks_per_rotation) * math.pi * 2 * self.wheel_radius 

        # print("l dist", l_dist)
        # print("r dist", r_dist)


        # print("previous left wheel ticks:", self.prev_l_ticks)
        # print("previous right wheel ticks:",  self.prev_r_ticks)


        # print("current left wheel ticks:", l_ticks, l_ticks - self.prev_l_ticks)
        # print("current right wheel ticks:", r_ticks, r_ticks - self.prev_r_ticks)



        self.prev_l_ticks = l_ticks
        self.prev_r_ticks = r_ticks


        # Basic kinematics 
        l_vel = l_dist / dt
        r_vel = r_dist / dt

        # print("left wheel velocity", l_vel, "m/s")
        # print("right wheel velocity", r_vel, "m/s")

        vel = (l_vel + r_vel) * 0.5
        dist = (l_dist + r_dist) * 0.5
        angular_vel = (l_vel - r_vel) / self.wheel_seperation

        # print("robot velocity", vel, "m/s")



        l_wheel_angular_pos = (((l_ticks - self.prev_l_ticks)/self.ticks_per_rotation) * 2 * math.pi)
        r_wheel_angular_pos = (((r_ticks - self.prev_r_ticks)/self.ticks_per_rotation) * 2 * math.pi)
        l_wheel_angular_vel =  l_wheel_angular_pos / dt
        r_wheel_angular_vel =  r_wheel_angular_pos / dt


        orientation = angular_vel * dt

        # if abs(orientation) < math.pow(10, -7): 
        #     direction = self.heading_ + orientation * 0.5
        #     self.x_ += dist * math.cos(direction)
        #     self.y_ += dist * math.sin(direction)
        #     self.heading_ += orientation
        # else:
        #     self.heading_ = math.fmod(self.heading_ + orientation, 2 * math.pi)
        #     self.x_ += dist * math.cos(self.heading_)
        #     self.y_ += dist * math.sin(self.heading_)

        self.x_ += dist * math.cos(self.heading_ + (orientation / 2.0))
        self.y_ += dist * math.sin(self.heading_ + (orientation / 2.0))
        self.heading_ = math.fmod(self.heading_ + orientation, 2 * math.pi)


        odomData = OdomData()
        odomData.robot_x = self.x_
        odomData.robot_y = self.y_
        odomData.robot_linear = vel
        odomData.robot_theta = self.heading_
        odomData.robot_angular = angular_vel
        odomData.angular_velocities = [l_wheel_angular_vel, r_wheel_angular_vel]
        odomData.angular_positions = [l_wheel_angular_pos, r_wheel_angular_pos]
        return odomData