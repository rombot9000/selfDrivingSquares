from collections import namedtuple
import numpy as np
from random import randint
from enum import Enum
from copy import copy, deepcopy

class dataType(Enum):
    center         = 1
    edges          = 2
    steeringEdge   = 3
    target         = 4

obstacle = namedtuple('Obstacle', ['distance', 'angle'])

class Shape:
    listOfAllShapes = []
    timeStep        = 0.05
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
        self.edges     = None
        self.calculateEdges()
        self.target    = np.array([randint(-100,100),randint(-100,100)])
        self.trajectory = {}
        for key in dataType:
            self.trajectory[key] = []
        self.addToTrajectory()
        #
        # Members for handling collisions etc
        #
        self.listOfObstacles = []
        self.isMoving        = True
        #
        # Set color of shape
        #
        self.color = '#{:06x}'.format(randint(0, 0xFFFFFF))
        print(self.color)

    
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
    
    def checkForCollisions(self, shape):
        print('Collision!')
        shape.stop()
        self.stop()
    
    def checkForObstacles(self):
        self.listOfObstacles = []
        scanningRange = 30
        for shape in self.listOfAllShapes:
            if shape is self:
                continue
            distance = np.linalg.norm(self.center - shape.center)
            if distance < (self.radius + shape.radius):
                self.checkForCollisions(shape)
            elif distance < scanningRange:
                relativeVector = shape.center - self.center
                angle = (np.arctan2(relativeVector[1], relativeVector[0]) - np.arctan2(self.direction[1], self.direction[0]))
                self.listOfObstacles.append(obstacle(distance, angle))
    
    def stop(self):
        self.isMoving = False
