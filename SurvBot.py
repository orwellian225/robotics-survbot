import numpy as np
import rospy as rp
import math as m

from Vec2 import Vec2

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Vector3, Twist
from gazebo_msgs.srv import GetModelState


"""
    Survillance Bot

    States
        * NAVIGATE -> Move towards the specified goal
        * PATROL -> Move along a cycle in the graph
        * PATHFIND -> Update the move queue to the Navigation Targat
        * IDLE -> Do Nothing
"""
class SurvBot:
    def __init__(self):
        self.state = "IDLE"
        rp.init_node("survbot")
        self.init_ros_subscribers()
        self.init_ros_publishers()

        self.nav_target = Vec2(0,0)
        self.position = Vec2(0,0)
        self.yaw = 0

        self.pos_sum_error = 0
        self.pos_prev_error = 0
        self.yaw_sum_error = 0
        self.yaw_prev_error = 0

        self.move_queue = []


    def init_ros_subscribers(self):
        self.state_sub = rp.Subscriber('/survbot/state', String, self.change_state) # Change state
        self.navgoal_sub = rp.Subscriber('/survbot/state/navigate/goal', Vector3, self.update_goal) # Change Navigate goal

    def init_ros_publishers(self):
        self.velocity_pub = rp.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.notify_pub = rp.Publisher('/survbot/notifications', String, queue_size=1)

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

        self.position = Vec2(coordinates.x, coordinates.y)
        self.yaw = m.atan2( 2 * ( rotation.w * rotation.z + rotation.x * rotation.y ), 1 - 2 * ( rotation.y**2 + rotation.z**2 ) )

        if self.state == "IDLE":
            self.state_idle()
        elif self.state == "PATHFIND":
            self.state_pathfind()
        elif self.state == "NAVIGATE":
            self.state_navigate()
        elif self.state == "PATROL":
            self.state_pathfind()

    def state_idle(self):
        pass

    def state_pathfind(self):
        pass

    def state_navigate(self):
        next_target = self.move_queue[0]
        reached_target = self.move_to(self.position, self.yaw, next_target)

        if reached_target:
            self.move_queue.pop(0)

        if len(self.move_queue) == 0:
            self.internal_change_state("IDLE")

    def state_patrol(self):
        next_target = self.move_queue[0]
        reached_target = self.move_to(self.position, self.yaw, next_target)

        if reached_target:
            self.move_queue.pop(0)
            self.move_queue.append(next_target)

    def internal_change_state(self, new_state):
        new_state = new_state.upper()
        if new_state == "IDLE" or new_state == "PATHFIND" or new_state == "NAVIGATE" or new_state == "PATROL":
            rp.loginfo("Changing state: %s -> %s", self.state, new_state)

            self.pos_sum_error = 0
            self.pos_prev_error = 0
            self.yaw_sum_error = 0
            self.yaw_prev_error = 0

            notification = String()
            notification.data = "state_change {0} -> {1}".format(self.state, new_state)
            self.notify_pub.publish(notification)

            self.state = new_state 
        else:
            rp.logwarn("Invalid state requested: %s does not exist", new_state)


    def change_state(self, data):
        new_state = str(data.data)
        self.internal_change_state(new_state)

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
            reset = Twist()
            self.velocity_pub.publish(reset)
            return True

        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        rp.loginfo("BOT Moving to (x,y): (%f,%f)", self.nav_target.x, self.nav_target.y)
        rp.loginfo("BOT Error: %f", pos_error)
        rp.loginfo("BOT forward Vel : %f", linear_vel)
        rp.loginfo("BOT turn Vel : %f", angular_vel)
        self.velocity_pub.publish(msg)

        return False

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