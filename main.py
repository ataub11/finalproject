import os

import numpy as np
import pybullet as p
import time
from tqdm import tqdm
from env import ClutteredPushGrasp
from utilities import YCBModels, Camera
from agent import BaseAgent

def box_demo():
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
def weird_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '021_bleach_cleanser', 'textured-decmp.obj'),
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
def round_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '011_banana', 'textured-decmp.obj'),
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

def combined_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
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

if __name__ == '__main__':
    #box_demo()
    #weird_demo()
    #round_demo()
    combined_demo()
