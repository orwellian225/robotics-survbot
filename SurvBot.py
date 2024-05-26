import numpy as np
import rospy as rp
import math as m

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Vector3, Twist
from gazebo_msgs.srv import GetModelState


class Vec2():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def magnitude(self):
        return m.sqrt(self.x**2 + self.y**2)

    def distance_to(self, other):
        return (self - other).magnitude()

    def angle_to(self, other):
        return m.acos( self.dot(other) / ( self.magnitude() * other.magnitude() ) )

"""
    Survillance Bot

    States
        * NAVIGATE -> Move towards the specified goal
        * PATROL -> Move along a cycle in the graph
        * IDLE -> Do Nothing
"""
class SurvBot:
    def __init__(self):
        self.state = "IDLE"
        rp.init_node("survbot")
        self.init_ros_subscribers()
        self.init_ros_publishers()

        self.nav_target = Vec2(0,0)
        self.pos_sum_error = 0
        self.pos_prev_error = 0
        self.yaw_sum_error = 0
        self.yaw_prev_error = 0


    def init_ros_subscribers(self):
        self.state_sub = rp.Subscriber('/survbot/state', String, self.change_state) # Change state
        self.navgoal_sub = rp.Subscriber('/survbot/state/navigate/goal', Vector3, self.update_goal) # Change Navigate goal
        self.patrol_sub = rp.Subscriber('/survbot/state/patrol/refresh', Empty, self.refresh_patrol_path) # Refresh the chosen patrol path

    def init_ros_publishers(self):
        self.velocity_pub = rp.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    def loop(self):
        rp_rate = rp.Rate(10)
        while not rp.is_shutdown():
            self.update()

            rp_rate.sleep()

    def update(self):
        coordinates, rotation = self.get_world_state()
        rp.loginfo("\n")
        rp.loginfo("BOT Status: %s", self.state)
        rp.loginfo("BOT Position (x,y,z): (%f, %f, %f)", coordinates.x, coordinates.y, coordinates.z)
        rp.loginfo("BOT Rotation (w,x,y,z): (%f, %f, %f, %f)", rotation.w, rotation.x, rotation.y, rotation.z)

        position = Vec2(coordinates.x, coordinates.y)
        yaw = m.atan2( 2 * ( rotation.w * rotation.z + rotation.x * rotation.y ), 1 - 2 * ( rotation.y**2 + rotation.z**2 ) )

        if self.state == "IDLE":
            pass
        elif self.state == "NAVIGATE":
            self.move_to(position, yaw, self.nav_target)
        elif self.state == "PATROL":
            pass

    def change_state(self, data):
        new_state = str(data.data)
        new_state = new_state.upper()

        if new_state == "IDLE" or new_state == "NAVIGATE" or new_state == "PATROL":
            rp.loginfo("Changing state: %s -> %s", self.state, new_state)
            self.state = new_state

            self.pos_sum_error = 0
            self.pos_prev_error = 0
            self.yaw_sum_error = 0
            self.yaw_prev_error = 0
        else:
            rp.logwarn("Invalid state requested: %s does not exist", new_state)

    def update_goal(self, data):
        coord_x = data.x
        coord_y = data.y
        self.nav_target = Vec2(coord_x, coord_y)
        rp.loginfo("Update Navigate Goal (x,y): (%f, %f)", coord_x, coord_y)

    def refresh_patrol_path(self):
        rp.loginfo("Refresh patrol path")

    def get_world_state(self):
        rp.wait_for_service("/gazebo/get_model_state")
        try:
            get_model_state = rp.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = get_model_state("mobile_base", "")
            coordinates = model_state.pose.position
            rotation = model_state.pose.orientation
            return (coordinates, rotation)

        except rp.ServiceException as e:
            rp.logerr("Service call failed: %s", e)
            return None

    def move_to(self, current_position, current_yaw, target_position):
        Kp_pos = 0.1
        Ki_pos = 0
        Kd_pos = 0

        Kp_yaw = 1
        Ki_yaw = 0
        Kd_yaw = 0

        pos_error = current_position.distance_to(target_position)
        self.pos_sum_error += pos_error
        pos_diff_error = pos_error - self.pos_prev_error
        self.pos_prev_error = pos_error

        target_direction = target_position - current_position
        bot_direction = Vec2( m.cos(current_yaw), m.sin(current_yaw) )
        yaw_error = target_direction.angle_to(bot_direction)
        self.yaw_sum_error += yaw_error
        yaw_diff_error = yaw_error - self.yaw_prev_error
        self.yaw_prev_error = yaw_error

        linear_vel = Kp_pos * pos_error + Ki_pos * self.pos_sum_error + Kd_pos * pos_diff_error
        angular_vel = Kp_yaw * yaw_error + Ki_yaw * self.yaw_sum_error + Kd_yaw * yaw_diff_error

        if pos_error < 0.1:
            to_idle = String()
            to_idle.data = "IDLE"
            self.change_state(to_idle)
            reset = Twist()
            self.velocity_pub.publish(reset)
            return

        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        rp.loginfo("BOT Moving to (x,y): (%f,%f)", self.nav_target.x, self.nav_target.y)
        rp.loginfo("BOT Error: %f", pos_error)
        rp.loginfo("BOT forward Vel : %f", linear_vel)
        rp.loginfo("BOT turn Vel : %f", angular_vel)
        self.velocity_pub.publish(msg)

def main():
    """
        Start the surveillance bot when this file is run
    """
    try:
        surv_bot = SurvBot()
        surv_bot.loop()
    except rp.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()