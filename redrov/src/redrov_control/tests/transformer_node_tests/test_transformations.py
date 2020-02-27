import pytest
import rospy
import time

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np


NAME = 'transfomer_tester'


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
            q = tf.transformations.quaternion_from_euler(p[3], p[4], p[5],  'rxyz')
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            for i in range(2):
                self.pub.publish(pose)
                time.sleep(0.2) # allow some time to let the topic to be published

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
            q = tf.transformations.quaternion_from_euler(p[3], p[4], p[5], 'rxyz')
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
            for i in range(2):
                self.pub.publish(odom)
                time.sleep(0.2) # allow some time to let the topic to be published

    return EulerOdomPublisher()

def pose_extraction(pose_data):
    pose = PoseStamped(dict(pose))
    result = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.x  
    return result

def test_can_publish_rpy_pose_and_receive_data(node, waiter, euler_pose_publish):
    waiter.condition = lambda msg: True
    rospy.Subscriber('/pose_ref', PoseStamped, waiter.callback)
    euler_pose_publish.publish([0, 0, 0, 0, 0, 0])
    waiter.wait(0.5) # give some time for the waiter to process the received data

    assert waiter.success 

def test_can_publish_rpy_odom_and_receive_data(node, waiter, euler_odom_publish):
    waiter.condition = lambda msg: True
    rospy.Subscriber('/BodyROV01/odom', Odometry, waiter.callback)
    euler_odom_publish.publish([0, 0, 0, 0, 0, 0])
    waiter.wait(0.5) # give some time for the waiter to process the received data

    assert waiter.success 

def test_can_receive_pose_error_data(node, waiter, euler_pose_publish, euler_odom_publish):
    waiter.condition = lambda msg: True
    rospy.Subscriber('/pose_error', PoseStamped, waiter.callback)
    euler_pose_publish.publish([0, 0, 0, 0, 0, 0])
    euler_odom_publish.publish([0, 0, 0, 0, 0, 0])
    waiter.wait(0.5) # give some time for the waiter to process the received data

    assert waiter.success 

def test_publish_right_rpy_pose_data(node, waiter, euler_pose_publish):
    waiter.condition = lambda msg: True
    rospy.Subscriber('/pose_ref', PoseStamped, waiter.callback)
    euler_pose_publish.publish([0, 0, 0, 0, 0, 0])
    waiter.wait(0.5) # give some time for the waiter to process the received data

    q = waiter.message[-1]
    assert np.allclose( pose_extraction(q), [0, 0, 0, 0, 0, 0, 1])



def test_pose_error_position_logic(node, waiter, euler_pose_publish, euler_odom_publish):
    waiter.condition = lambda msg: True
    rospy.Subscriber('/pose_error', PoseStamped, waiter.callback)
    euler_pose_publish.publish([0, 0, 0, 0, 0, np.pi/4])
    euler_odom_publish.publish([1, 1, 1, 0, 0, np.pi/4])
    waiter.wait(0.5) # give some time for the waiter to process the received data

    assert waiter.success 