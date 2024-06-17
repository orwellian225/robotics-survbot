import numpy as np
import rospy as rp
import math as m

from Graph import Graph
from PIDController import PIDController

from std_msgs.msg import String
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
        self.rate = 10

        self.pid = PIDController(
            '/mobile_base/commands/velocity',
            0.6, 0., 0., 1., 0., 0.
        )

        self.graph = Graph('maps/graph_data.csv', 'maps/map2.yaml', 'maps/map_data.png')

        self.nav_target = np.zeros(2)

        self.rotations = 1.
        self.rotation_speed = 0.3
        self.completed_rotations = 0.

        self.position = np.zeros(2)
        self.yaw = 0

        self.move_queue = []
        self.move_queue_idx = 0
        self.state_queue = []

        self.patrol_direction = 1

    def loop(self):
        rp_rate = rp.Rate(self.rate)
        while not rp.is_shutdown():
            self.update()

            rp_rate.sleep()

    def update(self):
        coordinates, rotation = self.get_world_state()
        if self.state != "IDLE":
            rp.loginfo("\n")
            rp.loginfo("BOT Status: %s", self.state)
            rp.loginfo("BOT Target (x,y): (%f,%f)", self.nav_target[0], self.nav_target[1])
            rp.loginfo("BOT Move Queue: %d / %d", self.move_queue_idx + 1, len(self.move_queue))
            # rp.loginfo("BOT Order Queue length: %d", len(self.order_queue))
            rp.loginfo("BOT Position (x,y,z): (%f, %f, %f)", coordinates.x, coordinates.y, coordinates.z)
            rp.loginfo("BOT Rotation (w,x,y,z): (%f, %f, %f, %f)", rotation.w, rotation.x, rotation.y, rotation.z)

        self.position = np.array([ coordinates.x, coordinates.y ])
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
            self.state_patrol()

    def change_state(self, new_state):
        new_state = new_state.upper()
        if new_state in all_states:
            rp.loginfo("Changing state: %s -> %s", self.state, new_state)

            self.pid.reset_pid()
            self.completed_rotations = 0.
            self.state = new_state 
        else:
            rp.logwarn("Invalid state requested: %s does not exist", new_state)

    def state_idle(self):
        pass

    def state_pathfind(self):
        end_result = self.graph.insert_vertex(self.nav_target)
        start_result = self.graph.insert_vertex(self.position)

        if start_result == -1:
            rp.logwarn("PATHFIND Start vertex is invalid")
            self.change_state("IDLE")
            return
        elif start_result == -2:
            rp.logwarn("PATHFIND Start vertex has no valid edge in graph")
            self.change_state("IDLE")
            return

        if end_result == -1:
            rp.logwarn("PATHFIND Target vertex is invalid")
            self.change_state("IDLE")
            return
        elif end_result == -2:
            rp.logwarn("PATHFIND Target vertex has no valid edge in graph")
            self.change_state("IDLE")
            return

        self.move_queue = self.graph.a_star(len(self.graph.vertices) - 1, len(self.graph.vertices) - 2)
        self.move_queue_idx = 0

        self.graph.remove_vertex(self.position)
        self.graph.remove_vertex(self.nav_target)

        self.change_state("IDLE")

    def state_navigate(self):
        if self.move_queue_idx == len(self.move_queue):
            rp.loginfo("Move Queue is Empty")
            self.move_queue = []
            self.move_queue_idx = 0
            self.change_state("IDLE")
            return

        pid_status = self.pid.apply_pid(self.position, self.yaw, self.move_queue[self.move_queue_idx])

        if pid_status == 1:
            self.move_queue_idx += 1
        elif pid_status == -1:
            rp.logwarn("SurvBot PID failed to move enough")
            self.change_state("IDLE")

    def state_scan(self):
        rotate_msg = Twist()
        rotate_msg.angular.z = self.rotation_speed
        self.completed_rotations += self.rotation_speed / self.rate
        rp.loginfo("BOT Completed Rotations: %f", self.completed_rotations)

        self.rotate_pub.publish(rotate_msg)

        if self.completed_rotations > self.rotations:
            rp.loginfo("Completed rotations")
            self.change_state("IDLE")


    def state_patrol(self):
        if self.move_queue_idx == len(self.move_queue):
            self.patrol_direction = -1
            self.move_queue_idx -= 1 # at +1 final vertex so just go back to the last vertex idx
        elif self.move_queue_idx == 0:
            self.patrol_direction = 1

        pid_status = self.pid.apply_pid(self.position, self.yaw, self.move_queue[self.move_queue_idx])

        if pid_status == 1:
            self.move_queue_idx += self.patrol_direction * 1
        elif pid_status == -1:
            rp.logwarn("SurvBot PID failed to move enough")
            self.change_state("IDLE")

    def update_state(self, data):
        new_state = str(data.data)
        self.change_state(new_state)

    def update_goal(self, data):
        coord_x = data.x
        coord_y = data.y
        self.nav_target = np.array([ coord_x, coord_y ])
        rp.loginfo("Update Navigate Goal (x,y): (%f, %f)", coord_x, coord_y)
        self.change_state("PATHFIND")

    def update_rotations(self, data):
        self.rotations = data.x * 2 * m.pi
        self.rotation_speed = data.y
        rp.loginfo("Update Scan parameters (number, speed): (%f, %f)", self.rotations, self.rotation_speed)

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
        self.rotate_sub = rp.Subscriber('/survbot/scan/rotations', Vector3, self.update_rotations) # Change Navigate goal

    def init_ros_publishers(self):
        # self.notify_pub = rp.Publisher('/survbot/notifications', String, queue_size=1)
        self.rotate_pub = rp.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
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
