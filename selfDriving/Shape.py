import numpy as np
from random import randint
from enum import Enum
from copy import copy, deepcopy


class dataType(Enum):
    center         = 1
    edges          = 2
    steeringEdge   = 3
    target = 4

class Shape:
    listOfAllShapes = []
    timeStep        = 0.5
    def __init__(self):
        #
        # Append instance of shape to list of shapes
        #
        self.listOfAllShapes.append(self)
        #
        # Set members needed for all shapes
        #
        self.center    = np.array([randint(-100,100),randint(-100,100)])
        self.direction = np.array([randint(-100,100),randint(-100,100)])
        self.direction = self.direction / np.linalg.norm(self.direction)
        self.edges          = [[None,None],[None,None]]
        self.calculateEdges()
        self.target    = np.array([randint(-100,100),randint(-100,100)])
        self.trajectory = {}
        for key in dataType:
            self.trajectory[key] = []
        self.addToTrajectory()
    
    def addToTrajectory(self):
        self.trajectory[dataType.center].append(deepcopy(self.center))
        self.trajectory[dataType.edges].append(deepcopy(self.edges))
        self.trajectory[dataType.target].append(deepcopy(self.target))
    
    # --------------------------------
    # Calculate hitbox, colisions, etc
    # --------------------------------
    def checkTarget(self):
        distanceToTarget = np.linalg.norm(self.target - self.center)
        if distanceToTarget < self.radius:
            print("Target position reached!\n")
            self.target = np.array([randint(-100,100),randint(-100,100)])
            self.trajectory[dataType.target].append(deepcopy(self.target))
            self.targetCounter += 1
    
    def checkForCollisions(self):
        for shape in self.listOfAllShapes:
            if shape is self:
                continue
            distance = np.linalg.norm(self.center - shape.center)
            if distance < (self.radius + shape.radius):
                print("Collision!")