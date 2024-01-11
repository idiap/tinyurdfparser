# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import PyTinyURDFParser

def print_robot_state(robot: PyTinyURDFParser.KDLRobot)->None:
    print("-----------------------------")
    print("Robot's Jacobian")
    print(robot.get_jacobian())
    print("Robot's Jacobian time derivative")
    print(robot.get_jacobian_derivative())
    print("EE Pos")
    print(robot.get_ee_pos())
    print("EE Vel")
    print(robot.get_ee_vel())
    print("EE orientation (quaternion)")
    print(robot.get_ee_orn_quat())
    print("Joint position")
    print(robot.get_q())
    print("Joint velocities")
    print(robot.get_dq())
    print("-----------------------------")


def main()->None:
    robot = PyTinyURDFParser.KDLRobot("./panda.urdf","panda_link0","panda_hand",[0]*7,[0]*7,[0]*3,[0]*3)
    print_robot_state(robot)

    cmd = [0,0.1,0,0.1,0,0,0]
    dt = 0.25

    print(f"Applying cmd={cmd} rad/s for {dt}s")
    robot.send_vel(dt,cmd,True)
    print_robot_state(robot)

if __name__=="__main__":
    main()
