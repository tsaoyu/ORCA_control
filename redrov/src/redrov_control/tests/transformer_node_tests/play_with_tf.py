import tf
from tf.transformations import quaternion_inverse, quaternion_multiply,quaternion_conjugate
import numpy as np
import time

def create_q_ref(e_angles):
    r, p, y = e_angles
    q = tf.transformations.quaternion_from_euler(r, p, y, 'sxyz')
    return q

def create_q_meas(e_angles):
    r, p, y = e_angles
    q = tf.transformations.quaternion_from_euler(r, p, y, 'sxyz')
    return q

def find_difference(q_mea, q_ref):
    q_error = quaternion_multiply( q_ref, quaternion_inverse(q_mea))
    return q_error

q_ref = create_q_ref([0, 0, 0])
q_mea = create_q_meas([np.pi/4, 0, np.pi/4])

def euler_angles(q_mea, q_ref):
    y, p, r = tf.transformations.euler_from_quaternion(find_difference(q_mea, q_ref), 'szyx')
    print('SZYX order:', r, p, y)

    r, p, y = tf.transformations.euler_from_quaternion(find_difference(q_mea, q_ref), 'sxyz')
    print('SXYZ order:', r, p, y)

    r, p, y = tf.transformations.euler_from_quaternion(find_difference(q_mea, q_ref), 'rxyz')
    print('RXYZ order:', r, p, y)

    y, p, r = tf.transformations.euler_from_quaternion(find_difference(q_mea, q_ref), 'rzyx')
    print('RZYX order:', r, p, y )

    print('_________________________________\n')

euler_angles(create_q_ref([0, 0, 0]), create_q_meas([np.pi/4, 0, np.pi/4]))
euler_angles(create_q_ref([np.pi/4, 0, 0]), create_q_meas([0, 0, np.pi/4]))
euler_angles(create_q_ref([1 , 0, 1]), create_q_meas([0, 0, 0]))

def simulation(e_mea_0, e_ref):
    q_ref = create_q_ref(e_ref)
    q_mea_0 = create_q_meas(e_mea_0)
    r, p, y = tf.transformations.euler_from_quaternion(find_difference(q_mea_0, q_ref), 'sxyz')
    q_mea = q_mea_0
    y_ref, p_ref, r_ref = e_ref
    y_mea, p_mea, r_mea = e_mea_0
    kp = 0.1
    while abs(y_mea - y_ref) > 0.01 or abs(p_mea - p_ref) > 0.01 or abs(r_mea - r_ref) > 0.01:
        r, p, y = tf.transformations.euler_from_quaternion(find_difference(q_mea, q_ref), 'sxyz')
        r_mea, p_mea, y_mea = tf.transformations.euler_from_quaternion(q_mea, 'sxyz')
        y_mea += kp * y
        p_mea += kp * p
        r_mea += kp * r
        q_mea = create_q_meas([r_mea, p_mea, y_mea])
        #print("CONTROl: {}, STATE: {}".format([r, p, y], [r_mea, p_mea, y_mea]))
        #time.sleep(0.05)
    print("Converged! \n", y_mea - y_ref, p_mea - p_ref, r_mea - r_ref)

simulation([0,0,np.pi/2], [0, 0 , 0]) # converge 

simulation([np.pi/4,0,np.pi/4], [0, 0 , 0]) # converge 

simulation([0,0,0], [np.pi/4, 0 , np.pi/4]) #  converge


simulation([1,0,0], [np.pi/4, 0 , np.pi/4]) #  converge

 


 