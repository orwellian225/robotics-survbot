import rospy as rp

from Vec2 import Vec2

from std_msgs.msg import String
from geometry_msgs.msg import Vector3

notifications_queue = []
navigation_target_queue = []

def received_notification(data):
    notifications_queue.append(str(data.data))

def main():
    print("Type exit to leave")
    new_state = str(input("New State: ")).upper()
    rp.init_node("survbot-controller")

    state_update_pub = rp.Publisher("/survbot/state", String, queue_size=1)
    goal_update_pub = rp.Publisher("/survbot/state/navigation/goal", Vector3, queue_size=1)
    notificatons_sub = rp.Subscriber("/survbot/notifications", String, received_notification)

    while new_state != "EXIT":
        if len(notifications_queue) != 0:
            if notifications_queue[0] == "state_change: PATHFIND -> NAVIGATE":
                goal_update = Vector3()
                goal_update.x = navigation_target_queue[0].x
                goal_update.y = navigation_target_queue[0].y
                goal_update.z = 0

                goal_update_pub.publish(goal_update)
                new_state = "NAVIGATE"


        if new_state == "PATHFIND":
            target_position_str = str(input("Target Position (x,y) : ")).split(",")
            target_position = Vec2(float(target_position_str[0]), float(target_position_str[1]))
            navigation_target_queue.append(target_position)

        msg = String()
        msg.data = new_state
        state_update_pub.publish(msg)

        new_state = str(input("New State: ")).upper()

    print("Exiting SurvBot Controller")


if __name__ == "__main__":
    main()