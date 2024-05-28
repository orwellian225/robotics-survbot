import numpy as np
import rospy as rp
import math as m

from Vec2 import Vec2
from Graph import Graph
from PIDController import PIDController

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Vector3, Twist
from gazebo_msgs.srv import GetModelState

"""
    Survillance Bot

    States
        * SCAN -> Rotate in place for a specified number of times
        * NAVIGATE -> Move towards the specified goal
        * PATROL -> Move along a cycle in the graph
        * PATHFIND -> Update the move queue to the Navigation Targat
        * IDLE -> Do Nothing
"""
all_states = ["IDLE", "PATHFIND", "NAVIGATE", "SCAN",  "PATROL"]
class SurvBot:
    def __init__(self):
        self.state = "IDLE"
        rp.init_node("survbot")
        self.init_ros_subscribers()
        self.init_ros_publishers()

        self.pid = PIDController(
            '/mobile_base/commands/velocity',
            0.1, 0., 0., 1., 0., 0.
        )

        self.nav_target = Vec2(0,0)
        self.rotations = 1.

        self.position = Vec2(0,0)
        self.yaw = 0

        self.move_queue = []
        self.state_queue = []

    def loop(self):
        rp_rate = rp.Rate(10)
        while not rp.is_shutdown():
            self.update()

            rp_rate.sleep()

    def update(self):
        coordinates, rotation = self.get_world_state()
        # if self.state != "IDLE":
        rp.loginfo("\n")
        rp.loginfo("BOT Status: %s", self.state)
        rp.loginfo("BOT Move Queue length: %d", len(self.move_queue))
        rp.loginfo("BOT Order Queue length: %d", len(self.move_queue))
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
        elif self.state == "SCAN":
            self.state_scan()
        elif self.state == "PATROL":
            self.state_pathfind()

    def change_state(self, new_state):
        new_state = new_state.upper()
        if new_state in all_states:
            rp.loginfo("Changing state: %s -> %s", self.state, new_state)

            self.pid.reset_pid()
            self.state = new_state 
        else:
            rp.logwarn("Invalid state requested: %s does not exist", new_state)

    def state_idle(self):
        pass

    def state_pathfind(self):
        pass

    def state_navigate(self):
        if len(self.move_queue) == 0:
            rp.loginfo("Move Queue is Empty")
            self.change_state("IDLE")
            return

        pid_status = self.move_to(self.position, self.yaw, self.move_queue[0])

        if pid_status == 1:
            self.move_queue.pop(0)
        elif pid_status == -1:
            rp.logwarn("SurvBot PID failed to move enough")
            self.change_state("IDLE")

    def state_scan(self):
        pass

    def state_patrol(self):
        pass

    def update_state(self, data):
        new_state = str(data.data)
        self.change_state(new_state)

    def update_goal(self, data):
        coord_x = data.x
        coord_y = data.y
        self.nav_target = Vec2(coord_x, coord_y)
        rp.loginfo("Update Navigate Goal (x,y): (%f, %f)", coord_x, coord_y)

    def update_rotations(self, data):
        pass

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

    def init_ros_subscribers(self):
        self.state_sub = rp.Subscriber('/survbot/state', String, self.update_state) # Change state
        self.navgoal_sub = rp.Subscriber('/survbot/navigate/goal', Vector3, self.update_goal) # Change Navigate goal
        self.rotate_sub = rp.Subscriber('/survbot/scan/rotations', Vector3, self.update_goal) # Change Navigate goal

    def init_ros_publishers(self):
        # self.notify_pub = rp.Publisher('/survbot/notifications', String, queue_size=1)
        pass


def main():
    """
        Start SurvBot when this file is run
    """
    try:
        surv_bot = SurvBot()
        surv_bot.loop()
    except rp.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
