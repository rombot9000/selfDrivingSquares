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

Obstacle = namedtuple('Obstacle', ['distance', 'angle', 'velocity', 'center', 'radius'])

class Shape:
    listOfAllShapes = []
    timeStep        = 0.05
    
    def moveAll(timeSpan):
        t = 0.0
        while t < (timeSpan - 0.5*Shape.timeStep):
            for shape in Shape.listOfAllShapes:
                shape.move()
            t += Shape.timeStep
    
    def __init__(self):
        #
        # Append instance of shape to list of shapes
        #
        Shape.listOfAllShapes.append(self)
        #
        # Set members needed for all shapes
        #
        self.center    = np.array([randint(-100,100),randint(-100,100)])
        self.direction = np.array([randint(-100,100),randint(-100,100)])
        self.direction = self.direction / np.linalg.norm(self.direction)
        self.velocity  = 0
        self.edges     = None
        self.calculateEdges()
        #self.target    = np.array([0,0])
        self.target   = np.array([randint(-100,100),randint(-100,100)])
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
        for shape in Shape.listOfAllShapes:
            if shape is self:
                continue
            distance = np.linalg.norm(self.center - shape.center)
            if distance < (self.radius + shape.radius):
                self.checkForCollisions(shape)
            elif distance < scanningRange:
                relativeVector = shape.center - self.center
                angle = (np.arctan2(relativeVector[1], relativeVector[0]) - np.arctan2(self.direction[1], self.direction[0]))
                while angle > np.pi:
                    angle -= 2*np.pi
                while angle < -np.pi:
                    angle += 2*np.pi
                #NOTE: for now we cheat by handing over the other objects's position and speed
                if abs(angle) <= np.pi/2:
                    self.listOfObstacles.append(Obstacle(distance, angle, shape.velocity*shape.direction, shape.center, shape.radius))
    
    def stop(self):
        self.isMoving = False
        self.velocity = 0.0
