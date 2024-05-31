import math as m
import rospy as rp
from Vec2 import Vec2

from geometry_msgs.msg import Twist

class PIDController:
    def __init__(self, velocity_topic,
        position_Kp, position_Ki, position_Kd,
        yaw_Kp, yaw_Ki, yaw_Kd 
    ):

        self.reset_pid()
        self.velocity_pub = rp.Publisher(velocity_topic, Twist, queue_size=1)

        self.yaw_Kp = yaw_Kp
        self.yaw_Ki = yaw_Ki
        self.yaw_Kd = yaw_Kd

        self.position_Kp = position_Kp
        self.position_Ki = position_Ki
        self.position_Kd = position_Kd

    def reset_pid(self):
        self.position_sum_error = 0.
        self.position_prev_error = 0.

        self.yaw_sum_error = 0.
        self.yaw_prev_error = 0.

    def apply_pid(self, current_position, current_yaw, target_position):
        """
            Apply PID Movement to SurvBot
            
            Return Values:
                -1 => Error has occurred
                0 => Not reached target
                1 => Reached target
        """

        pos_error = current_position.distance_to(target_position)
        self.position_sum_error += pos_error
        pos_diff_error = pos_error - self.position_prev_error
        self.position_prev_error = pos_error

        target_direction = target_position - current_position
        bot_direction = Vec2( m.cos(current_yaw), m.sin(current_yaw) )
        yaw_error = target_direction.angle_to(bot_direction)
        self.yaw_sum_error += yaw_error
        yaw_diff_error = yaw_error - self.yaw_prev_error
        self.yaw_prev_error = yaw_error

        linear_vel = self.position_Kp * pos_error + self.position_Ki * self.position_sum_error + self.position_Kd * pos_diff_error
        angular_vel = self.yaw_Kp * yaw_error + self.yaw_Ki * self.yaw_sum_error + self.yaw_Kd * yaw_diff_error

        msg = Twist()
        if angular_vel < 1e-3:
            msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        rp.loginfo("PID Moving to (x,y): (%f,%f)", target_position.x, target_position.y)
        rp.loginfo("PID Distance Error: %f", pos_error)
        rp.loginfo("PID Turn Error: %f", yaw_error)
        rp.loginfo("PID forward Velocity : %f", linear_vel)
        rp.loginfo("PID Turn Velocity: %f", angular_vel)

        # If SurvBot hasn't moved enough, then exit with an error
        # if pos_diff_error < 1e-3 and yaw_diff_error < 1e-3:
        #     return -1

        if pos_error < 0.1:
            reset = Twist()
            self.velocity_pub.publish(reset)
            return 1

        self.velocity_pub.publish(msg)

        return 0 