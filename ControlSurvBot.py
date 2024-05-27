import rospy as rp
import time

from Vec2 import Vec2

from std_msgs.msg import String
from geometry_msgs.msg import Vector3

def main():
    print("Type exit to leave")
    new_state = str(raw_input("New State: ")).upper()
    rp.init_node("survbot_controller")

    state_update_pub = rp.Publisher("/survbot/state", String, queue_size=1)
    goal_update_pub = rp.Publisher("/survbot/state/navigate/goal", Vector3, queue_size=1)
    # notificatons_sub = rp.Subscriber("/survbot/notifications", String, received_notification)

    while new_state != "EXIT":
        if new_state == "PATHFIND":
            target_position_str = str(raw_input("Target Position (x,y): ")).split(",")
            target_position = Vec2(float(target_position_str[0]), float(target_position_str[1]))
            print("Pathfinding to {0},{1}".format(target_position.x, target_position.y))

            goal_update = Vector3()
            goal_update.x = target_position.x
            goal_update.y = target_position.y
            goal_update.z = 0

            goal_update_pub.publish(goal_update)
            new_state = "PATHFIND"
            time.sleep(2)

        msg = String()
        msg.data = new_state
        state_update_pub.publish(msg)

        new_state = str(raw_input("New State: ")).upper()

    print("Exiting SurvBot Controller")


if __name__ == "__main__":
    main()