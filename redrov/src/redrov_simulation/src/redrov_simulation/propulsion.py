from __future__ import division
import numpy as np

allocation_matrix = np.array([[-1,  1,  0,   1],
                              [-1, -1,  0,  -1],
                              [ 1,  1,  0,  -1],
                              [ 1, -1,  0,   1],
                              [ 0,  0, -1,   0],
                              [ 0,  0, -1,   0]])  


fit_param = np.array([-9.61680578e+03, -6.99048595e+02,  4.43322720e+04,  2.47918407e+03,
    -8.51058600e+04, -3.54208066e+03,  8.80005365e+04,  2.63030018e+03,
    -5.30120789e+04, -1.10323287e+03,  1.88403735e+04,  2.74935028e+02,
    -3.85071008e+03, -4.63403410e+01,  4.54202447e+02,  1.20375096e+01,
    3.77929694e+00,  0.00000000e+00])

prop_matrix = np.array(
                [[0.7071067811847431, 0.7071067811847433, -0.7071067811919605, -0.7071067811919607, 0.0, 0.0],
                [-0.7071067811883519, 0.7071067811883519, -0.7071067811811347, 0.7071067811811349, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0,  1.0000000000000002, 1.0000000000000002],
                [0.051265241635893875, -0.051265241635893875, 0.051265241635893875, -0.051265241635893875, -0.11050000000000003, -0.11050000000000003],
                [0.051265241635893875, 0.051265241635893875, -0.051265241635893875, -0.051265241635893875, 0.0024999999999744805, -0.0024999999999744805],
                [-0.166523646969496, 0.16652364696949604, 0.17500892834341342, -0.17500892834341347, 0.0, 0.0]])

def prop_character(effort):

    if abs(effort) > 100:
        print("Effort out of range!")
        return
        
    command = effort / 100.
    force  = np.polyval(fit_param, command)
    return force

def BlueROV2Input(wrench_input, frame="4dof"):
    if len(wrench_input) != 4:
        raise NotImplementedError

    effort = np.dot(allocation_matrix, wrench_input)
    effort = np.clip(effort, -100, 100)
    # print(effort)
    u = [prop_character(i) for i in effort]
    # print(u)                                  
    tau = np.dot(prop_matrix, u)
    return tau


if __name__ == "__main__":
  
    print(BlueROV2Input([50,0,0,0]))
    print(BlueROV2Input([-50,0,0,0]))
    print(BlueROV2Input([0,50,0,0]))
    print(BlueROV2Input([0,0,50,0]))
    print(BlueROV2Input([0,0,0,50]))