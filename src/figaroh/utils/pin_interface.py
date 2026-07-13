import pinocchio as pin
import numpy as np

def init_robot(robot):
    import pinocchio as pin

    pin.framesForwardKinematics(robot.model, robot.data, robot.q0)
    pin.updateFramePlacements(robot.model, robot.data)


def calc_torque(N, robot, q, v, a):
    tau = np.zeros(robot.model.nv * N)
    for i in range(N): # 第i组路点
        for j in range(robot.model.nv): #第j个关节
            tau[j * N + i] = pin.rnea(robot.model, robot.data, q[i, :], v[i, :], a[i, :])[j]
    return tau