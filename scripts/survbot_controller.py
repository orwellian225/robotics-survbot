import time

import rospy as rp

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3

def print_help():
    print("How to use:")
    print("\t1. Enter an instruction")
    print("\t2. Enter the required parameters")
    print("")

    print("Available Instructions:")
    print("-----------------------")
    print("\tEXIT - shutdown the controller")
    print("\tHELP - print this guide")
    print("\tREPEAT - repeat the previous instruction - will not reissue previous parameters")
    print("-----------------------")
    print("\tSTOP - Stop SurvBot from doing anything")
    print("\tSTART - Make SurvBot move towards its current navigation target")
    print("\tTARGET - Change SurvBot's navigation target")
    print("\tROTATE - Change the number of rotations in a scan")
    print("\tSCAN - Make SurvBot scan its current surroudings")
    print("\tPATROL - Make SurvBot follow a patrol path")
    print("-----------------------")
    print("\tMOVE-TO - Change SurvBot's navigation target, and then move towards it")
    print("\tSCAN-FOR - Change SurvBot's rotation count, and then perform a scan")

instructions = [
    "EXIT", "HELP", "REPEAT",
    "STOP", "START", "SCAN", "PATROL", "TARGET", "ROTATE", "MOVE-TO", "SCAN-FOR"
]

def handle_instruction(instruction, publishers, previous_instruction):
    if instruction not in instructions:
        print("Invalid Instruction")
        print("Enter HELP to see a list of instructions")
    
    if instruction == "EXIT":
        print("Exiting SurvBot Controller")
        exit(0)

    elif instruction == "HELP":
        print_help()

    elif instruction == "REPEAT":
        print("Repeating previous instruction", previous_instruction)
        handle_instruction(previous_instruction, publishers, previous_instruction)

    elif instruction == "STOP":
        msg = String()
        msg.data = "IDLE"
        publishers['change_state'].publish(msg)
        print("\tStopping SurvBot")

    elif instruction == "START":
        msg = String()
        msg.data = "NAVIGATE"
        publishers['change_state'].publish(msg)
        print("\tStarting SurvBot")

    elif instruction == "SCAN":
        msg = String()
        msg.data = "SCAN"
        publishers['change_state'].publish(msg)
        print("\tStarting scan")

    elif instruction == "PATROL":
        msg = String()
        msg.data = "PATROL"
        publishers['change_state'].publish(msg)
        print("\tStarting patrol")

    elif instruction == "TARGET":
        target_str = str(raw_input("\tTarget Position (x,y): ")).split(",")

        target_msg = Vector3()
        target_msg.x = float(target_str[0]) 
        target_msg.y = float(target_str[1]) 
        target_msg.z = 0.0

        publishers['change_goal'].publish(target_msg)
        print("\tSetting navigation target ({0},{1})".format(target_str[0], target_str[1]))

    elif instruction == "ROTATE":
        rotations = float(raw_input("\tNumber of rotations (x): "))
        msg = Float32()
        msg.data = rotations 

        publishers['change_scan'].publish(msg)
        print("\tSetting number of rotations ({0})".format(rotations))

    elif instruction == "MOVE-TO":
        target_str = str(raw_input("\tTarget Position (x,y): ")).split(",")

        target_msg = Vector3()
        target_msg.x = float(target_str[0]) 
        target_msg.y = float(target_str[1]) 
        target_msg.z = 0.0

        publishers['change_goal'].publish(target_msg)

        time.sleep(1)
        state_msg = String()
        state_msg.data = "NAVIGATE"
        publishers['change_state'].publish(state_msg)
        print("\tMoving to {0},{1}".format(target_str[0], target_str[1]))

    elif instruction == "SCAN-FOR":
        rotations = float(raw_input("\tNumber of rotations (x): "))
        msg = Float32()
        msg.data = rotations 

        publishers['change_scan'].publish(msg)

        time.sleep(1)
        state_msg = String()
        state_msg.data = "SCAN"
        publishers['change_state'].publish(state_msg)
        print("\tScanning for {0} rotations".format(rotations))

    return instruction

def main():
    print("SurvBot Controller")
    print("------------------\b")
    print_help()

    rp.init_node("survbot_controller")

    next_instruction = ""
    prev_instruction = ""

    ros_pubs = {
        'change_state': rp.Publisher('/survbot/state', String, queue_size=1),
        'change_goal': rp.Publisher('/survbot/navigate/goal', Vector3, queue_size=1),
        'change_scan': rp.Publisher('/survbot/scan/rotations', Float32, queue_size=1),
    }

    while next_instruction != "EXIT":
        next_instruction = str(raw_input("Instruction: ")).upper()
        prev_instruction = handle_instruction(next_instruction, ros_pubs, prev_instruction)

if __name__ == "__main__":
    main()