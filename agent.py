from tqdm import tqdm
import matplotlib.pyplot as plt
from collections import namedtuple
import pybullet as p
import random





class BaseAgent(object):
    def potentialAct(objnum, allMovedobjs):
        targetCenterX, targetCenterY = .75, .05
        potentialMoves = dict()
        targetX = targetCenterX - .5
        targetY = targetCenterY - .5
        while (targetX < (targetCenterX+.5) and targetY < (targetCenterY+.5)):
            score = BaseAgent.scoreLocations(objnum, allMovedobjs, targetX, targetY)
            pos = [targetX, targetY, 1.2]
            potentialMoves[score] = pos
            targetX += .05
            targetY += .05
        print(potentialMoves)
        return potentialMoves
    def scoreLocations(objnum, allMovedobjs, x, y):
        objs = objnum
        moved = allMovedobjs
        reward = 0
        for num in range(objs):
            pos, ori = p.getBasePositionAndOrientation(moved[num-1])
            checkX, checkY, checkZ = pos[:3]
            if(checkX -.01 <= x <= checkX +.01):
                reward -= .25
            if((checkY -.01 <= y <= checkY +.01)):
                reward -= .5
        return reward
    def pickAction(objnum, allMoved):
        obj = objnum
        moved = allMoved
        allMoves = BaseAgent.potentialAct(obj, moved)
        scores = allMoves.keys()
        best = max(scores)
        targets = allMoves.get(best)
        return targets






