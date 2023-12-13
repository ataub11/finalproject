import os

import numpy as np
import pybullet as p

from tqdm import tqdm
from env import ClutteredPushGrasp
from utilities import YCBModels, Camera
from statistics import mean

def heuristic_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '061_foam_brick', 'textured-decmp.obj'),
    )
    camera = Camera((0, -0.5, 1.5), 0.1, 5, (320, 320), 40)

    env = ClutteredPushGrasp(ycb_models, camera, vis=True, num_objs=3, gripper_type='85')
    p.resetDebugVisualizerCamera(2.0, -270., -60., (0., 0., 0.))
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Shadows on/off

    _w, _h, rgb, depth = env.reset()
    step_cnt = 0
    while True:
        pos, orient = p.getBasePositionAndOrientation(env.obj_ids[0])
        x, y,z = pos[:3]
       

        #p.addUserDebugLine([x, y, 0], [x, y, z], [0, 1, 0])
        #p.addUserDebugLine([x, y, z], [x, y, z+0.05], [1, 0, 0])

        (rgb, depth, seg), reward, done, info = env.step((x, y, z), 1, 'grasp')

        print('Step %d, grasp at %.2f,%.2f,%.2f, reward %f, done %s, info %s' %
              (step_cnt, x, y, z, reward, done, info))
        step_cnt += 1
        # time.sleep(3)


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
