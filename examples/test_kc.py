# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
#
# SPDX-License-Identifier: GPL-3.0-only

import PyTinyURDFParser

def main()->None:
    
    q_init = [0]*7
    dq_init = [0]*7
    
    base_link = "panda_link0"
    tip_link = "panda_link8"
    urdf_path = "./panda.urdf"

    robot = PyTinyURDFParser.KDLRobot(urdf_path,base_link,tip_link,q_init,dq_init,[0]*3,[0]*3)
    robot.set_conf([0.31748759,0.57746106,-0.2330027,-1.89712516,-1.04131095,1.98633811,0.80193143],[0]*7,True)
    
    print("Obtained position: ",robot.get_ee_pos())
    print("Obtained orientation: ",robot.get_ee_orn_quat())


if __name__=="__main__":
    main()
