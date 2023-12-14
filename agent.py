from tqdm import tqdm
import matplotlib.pyplot as plt
from collections import namedtuple
import pybullet as p
import random





class BaseAgent(object):
    def potentialAct(self, objnum, allMovedobjs):
        targetCenter = p.getBasePositionAndOrientation(self.tablebID)
        targetCenterX, targetCenterY, targetCenterZ = targetCenter[:3]
        potentialMoves = dict()
        targetX = targetCenterX - .5
        targetY = targetCenterY - .5
        while (targetX < (targetCenterX+.5) and targetY < (targetCenterY+.5)):
            score = BaseAgent.scoreLocations(self, objnum, allMovedobjs, targetX, targetY)
            pos = [targetX, targetY]
            potentialMoves.update(score, pos)
            print(potentialMoves)
        return potentialMoves
    def scoreLocations(self, objnum, allMovedobjs, x, y):
        objs = objnum
        moved = allMovedobjs
        reward = 0
        for num in range(objs):
            pos, ori = p.getBasePositionAndOrientation(moved[num-1])
            checkX, checkY, checkZ = pos[:3]
            if(checkX -.001 <= x <= checkX +.001):
                reward -= .25
            if((checkY -.001 <= y <= checkY +.001)):
                reward -= .5
        return reward
    def pickAction(self, objnum, allMoved):
        obj = objnum
        moved = allMoved
        allMoves = BaseAgent.potentialAct(self, obj, moved)
        scores = allMoves.keys()
        best = max(scores)
        targets = allMoves.get(best, [])
        return random.choice(targets)






