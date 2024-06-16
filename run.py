import time
from multiprocessing import Process

import rospy as rp
from src.SurvBot import SurvBot

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3

def loop_surv():
    try:
        surv = SurvBot()
        surv.loop()
    except rp.ROSInterruptException:
        pass

goal_topic =  rp.Publisher('/survbot/navigate/goal', Vector3, queue_size=1)
state_topic = rp.Publisher('/survbot/state', String, queue_size=1)

surv_process = Process(target=loop_surv, args=())
surv_process.start()

rp.init_node("survbot_runner")
target_str = str(raw_input("Target Position (x,y): ")).split(",")
target_msg = Vector3()
target_msg.x = float(target_str[0]) 
target_msg.y = float(target_str[1]) 
target_msg.z = 0.0

goal_topic.publish(target_msg)
time.sleep(1)

state_msg = String()
state_msg.data = "NAVIGATE"
state_topic.publish(state_msg)

surv_process.join()