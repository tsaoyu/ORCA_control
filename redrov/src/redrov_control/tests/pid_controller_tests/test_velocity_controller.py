import pytest
import rospy
import time

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Wrench
import numpy as np
import yaml

NAME = "controller_tester"

@pytest.fixture
def node():
    rospy.init_node(NAME, anonymous=True)

@pytest.fixture
def waiter():
    class Waiter(object):
        def __init__(self):
            self.received = []
            self.message = []
            self.condition = lambda x: False

        @property
        def success(self):
            return True in self.received

        def callback(self, data):
            self.received.append(self.condition(data))
            self.message.append(data)

        def wait(self, timeout):
            timeout_t = time.time() + timeout
            while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                time.sleep(0.1)

        def reset(self):
            self.received = []
    return Waiter()


@pytest.fixture
def euler_pose_publish():
    class EulerPosePublisher():
        def __init__(self):
            self.pub = rospy.Publisher('/pose_ref', PoseStamped, queue_size=10)

        def publish(self, pose_list):
            p = pose_list
            pose = PoseStamped()

            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p[:3]
            q = tf.transformations.quaternion_from_euler(p[3], p[4], p[5], 'sxyz')
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            for i in range(2):
                self.pub.publish(pose)
                time.sleep(0.1) # allow some time to let the topic to be published

    return EulerPosePublisher()

@pytest.fixture
def euler_odom_publish():
    class EulerOdomPublisher():
        def __init__(self):
            self.pub = rospy.Publisher('/BodyROV01/odom', Odometry, queue_size=10)

        def publish(self, pose_list):
            p = pose_list
            odom = Odometry()

            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = p[:3]
            q = tf.transformations.quaternion_from_euler(p[3], p[4], p[5], 'sxyz')
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
            for i in range(2):
                self.pub.publish(odom)
                time.sleep(0.1) # allow some time to let the topic to be published

    return EulerOdomPublisher()

def pose_extraction(pose_data):
    pose = yaml.load(str(pose_data))
    result = [pose['pose']['position']['x'], pose['pose']['position']['y'], pose['pose']['position']['y'],\
              pose['pose']['orientation']['x'], pose['pose']['orientation']['y'], pose['pose']['orientation']['z'], pose['pose']['orientation']['w']]  
    return result

def twist_extraction(twist_data):
    twist = yaml.load(str(twist_data))
    result = [twist['force']['x'], twist['force']['y'], twist['force']['z'], \
              twist['torque']['x'], twist['torque']['y'], twist['torque']['z']]  
    return result

def wrench_extraction(wrench_data):
    wrench = yaml.load(str(wrench_data))
    result = [wrench['linear']['x'], wrench['linear']['y'], wrench['linear']['z'], \
              wrench['angular']['x'], wrench['angular']['y'], wrench['angular']['z']]  
    return result


def test_can_publish_cmd_wrench(node, waiter, euler_pose_publish, euler_odom_publish):
    waiter.condition = lambda msg: True
    rospy.Subscriber('/cmd_wrench', Wrench, waiter.callback)
    euler_pose_publish.publish([0, 0, 0, 0, 0, 0])
    euler_odom_publish.publish([0, 0, 0, 0, 0, 0])
    waiter.wait(0.5) # give some time for the waiter to process the received data

    assert waiter.success 


def test_can_publisher_correct_wrench(node, waiter, euler_pose_publish, euler_odom_publish):
    waiter.condition = lambda msg: True
    rospy.Subscriber('/cmd_wrench', Twist, waiter.callback)
    euler_pose_publish.publish([0, 0, 0, 0, 0, 0])
    euler_odom_publish.publish([1, 1, 1, 0, 0, 0])
    waiter.wait(0.5) # give some time for the waiter to process the received data

    twist = twist_extraction(waiter.message[-1])
    assert np.allclose(twist, [-1, -1, -1, 0, 0, 0])

    euler_odom_publish.publish([2, 2, 2, 0, 0, 0])
    waiter.wait(0.5) # give some time for the waiter to process the received data
    twist = twist_extraction(waiter.message[-1])
    assert np.allclose(twist, [-2, -2, -1, 0, 0, 0])
