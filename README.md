## Incorporating Previous Action Effects into Planning for Next Goal


This repo is under active development. Issues / PRs are welcomed.


## Highlights

- UR5 arm with end-effector 6D IK (Position [X Y Z] and Orientation [R P Y])
- Enhanced Robotiq-85 / 140 gripper, with precise position control and experimental torque control
- Built-in YCB models loader (and obj models after decomposition)
- Gym-styled API, making it suitable for Reinforcement Learning in the field of push-and-grasp
- Demos for moving boxes, weirdly shaped objects, round objects, and a mix of all three object types

## Prerequisite
- Python 3
- PyBullet

## Run

You can try this repo with any demo.
```[Python]
python main.py
```
## References 

https://github.com/matafela/pybullet_grasp_annotator_robotiq_85

https://github.com/zswang666/pybullet-playground

https://github.com/ros-industrial/robotiq

https://github.com/Alchemist77/pybullet-ur5-equipped-with-robotiq-140

https://github.com/ElectronicElephant/pybullet_ur5_robotiq 

I do not claim copyright for any model files under this repo.
