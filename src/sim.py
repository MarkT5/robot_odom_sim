#!/usr/bin/env python
import rospy
import time
import math
from tf.transformations import *
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from robot_odom_sim.msg import Encoder
from nav_msgs.msg import Odometry, Path




class Wheel:
    def __init__(self):
        self.k = 4
        self.integral = 0
        self.curr_vel = 0
        self.target_vel = 0
        self.T = 0.1

    def step(self, dt):
        err = self.target_vel - self.curr_vel
        self.integral += err * dt
        self.curr_vel += err * self.k / (abs(self.integral) * self.T + 1) * dt


def ctrl_callback(data):
    global robot_com, robot_wheels
    robot_com = data
    robot_wheels[0].target_vel = data.linear.x - data.angular.z*5
    robot_wheels[1].target_vel = data.linear.x + data.angular.z*5


def listener():
    # rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, ctrl_callback)


def talker():
    global seq, robot_com, robot_wheels, robot_odom, last_t, L, r, pos, counter
    enc_pub = rospy.Publisher('encoder', Encoder, queue_size=10)

    odom_pub = rospy.Publisher('odom_sim', Odometry, queue_size=10)
    pos_pub = rospy.Publisher('pos_sim', PoseStamped, queue_size=10)
    path_pub = rospy.Publisher('path_sim', Path, queue_size=10)

    rospy.init_node('sim_node', anonymous=False)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        dt = time.time()-last_t
        last_t = time.time()
        robot_wheels[0].step(dt)
        robot_wheels[1].step(dt)
        dl = robot_wheels[0].curr_vel * dt / (2*math.pi)
        dr = robot_wheels[1].curr_vel * dt / (2*math.pi)
        robot_odom[0] += dl*4096
        robot_odom[1] += dr*4096
        dl *= r
        dr *= r

        dfov = (dr+dl)/2
        dang = math.atan2((dr-dl), L)
        pos[0] += dfov*math.cos(pos[2])
        pos[1] += dfov*math.sin(pos[2])
        pos[2] += dang


        posS = PoseStamped()
        posS.header.seq = seq
        posS.header.frame_id = "Odom_frame"
        posS.header.stamp = rospy.Time.now()
        posS.pose.position.x = pos[0]*100
        posS.pose.position.y = pos[1]*100
        x, y, z, w = quaternion_from_euler(0,0,pos[2])
        posS.pose.orientation.x = x
        posS.pose.orientation.y = y
        posS.pose.orientation.z = z
        posS.pose.orientation.w = w
        pos_pub.publish(posS)
        if counter % 10 == 0:
            poses.append(posS)
            path = Path()
            path.header.seq = seq
            path.header.frame_id = "Odom_frame"
            path.header.stamp = rospy.Time.now()
            path.poses = poses
            path_pub.publish(path)

        odom_msg = Odometry()
        odom_msg.header.seq = seq
        odom_msg.header.frame_id = "Odom_frame"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = pos[0]
        odom_msg.pose.pose.position.y = pos[1]
        odom_msg.pose.pose.orientation.z = pos[2]
        odom_pub.publish(odom_msg)

        e_msg = Encoder()
        e_msg.head.seq = seq
        e_msg.head.frame_id = "Encoder_frame"
        e_msg.head.stamp = rospy.Time.now()
        e_msg.left = int(robot_odom[0])
        e_msg.right = int(robot_odom[1])
        enc_pub.publish(e_msg)

        seq += 1
        rate.sleep()


if __name__ == '__main__':
    L = 0.287
    r = 0.033
    poses = []
    seq = 0
    counter = 0
    last_t = time.time()
    robot_com = Twist()
    robot_wheels = [Wheel(), Wheel()]
    robot_odom = [0, 0]
    pos = [0, 0, 0]
    try:
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass
