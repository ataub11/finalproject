import os

import numpy as np
import pybullet as p
import time
from tqdm import tqdm
from env import ClutteredPushGrasp
from utilities import YCBModels, Camera
from agent import BaseAgent

def heuristic_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '061_foam_brick', 'textured-decmp.obj'),
    )
    camera = Camera((0, -0.5, 1.5), 0.1, 5, (320, 320), 40)
    objnum = 4

    env = ClutteredPushGrasp(ycb_models, camera, vis=True, num_objs=objnum, gripper_type='85')
    p.resetDebugVisualizerCamera(2.0, -270., -60., (0., 0., 0.))
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Shadows on/off

    _w, _h, rgb, depth = env.reset()
    step_cnt = 0
    objnum -=1
    while True:
        if (objnum >= 0):
            pos, orient = p.getBasePositionAndOrientation(env.obj_ids[objnum])
            x, y,z = pos[:3]
            if step_cnt == 0:
                env.step((x, y, z), 1, 'grasp')
                time.sleep(2)
            else:
                target = BaseAgent.pickAction(objnum, env.successful_obj_ids)
                pos, orient = p.getBasePositionAndOrientation(env.obj_ids[objnum])
                x,y,z = pos[:3]
                env.Agentstep(pos, 1, 'grasp', target)

            step_cnt += 1
            objnum -= 1
        else:
            time.sleep(3)


def user_control_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', "036_wood_block", 'textured-decmp.obj'),
    )
    camera = Camera((0, -0.5, 1.5), 0.1, 5, (320, 320), 40)

    env = ClutteredPushGrasp(ycb_models, camera, vis=True, num_objs=5, gripper_type='85')
    p.resetDebugVisualizerCamera(2.0, -270., -60., (0., 0., 0.))
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Shadows on/off
    p.addUserDebugLine([0, -0.5, 0], [0, -0.5, 1.1], [0, 1, 0])

    env.reset()
    while True:
        env.step(None, None, None, True)

        # key control
        keys = p.getKeyboardEvents()
        # key "Z" is down and hold
        if (122 in keys) and (keys[122] == 3):
            print('Grasping...')
            if env.close_gripper(check_contact=True):
                print('Grasped!')
        # key R
        if 114 in keys:
            env.open_gripper()
        # time.sleep(1 / 120.)


if __name__ == '__main__':
    #user_control_demo()
    heuristic_demo()
