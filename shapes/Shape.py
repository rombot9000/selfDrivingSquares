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
    edge_1         = 5
    edge_2         = 6
    edge_3         = 7
    edge_4         = 8

Obstacle = namedtuple('Obstacle', ['distance', 'angle', 'velocity', 'center', 'radius'])

greyAsHex = '#A0A0A0'

class Shape:
    __LIST = []
    _TIMESTEP = 0.05
    
    @staticmethod
    def moveAll(timeSpan):
        '''
        Iterate move method for all shapes for given period
        '''
        t = 0.0
        while t < (timeSpan - 0.5*Shape._TIMESTEP):
            for shape in Shape.__LIST:
                if shape.isMoving: shape.move()
            t += Shape._TIMESTEP
    
    def __init__(self):
        #
        # Append instance of shape to list of shapes
        #
        Shape.__LIST.append(self)
        # public
        self.shapeID   = len(Shape.__LIST)
        self.isMoving  = True
        self.center    = np.array([randint(-100,100),randint(-100,100)])
        self.edges     = []
        self.radius    = 0
        self.color = '#{:06x}'.format(randint(0, 0xFFFFFF))
        self.target    = np.array([randint(-100,100),randint(-100,100)])
        self.targetCounter   = 0
        self.trajectory = {}
        for key in dataType:
            self.trajectory[key] = []
        # protected
        self._direction = np.array([randint(-100,100),randint(-100,100)])
        self._direction = self._direction / np.linalg.norm(self._direction)
        self._velocity  = 0
        self._listOfObstacles = []

        # call init funcs
        self._defineEdges()
        self._calculateRadius()
        self._addToTrajectory()
    
    def stop(self):
        self.isMoving = False
        self._velocity = 0.0
        self.color    = greyAsHex
    
    def move(self):
        '''
        Needs to be define in child class
        '''
        return
    
    def _log(self, __VAR_ARGS__):
        print("SID {}: ".format(self.shapeID), __VAR_ARGS__)
    
    def _addToTrajectory(self):
        self.trajectory[dataType.center].append(deepcopy(self.center))
        self.trajectory[dataType.edge_1].append(deepcopy(self.edges[0]))
        self.trajectory[dataType.edge_2].append(deepcopy(self.edges[1]))
        self.trajectory[dataType.edge_3].append(deepcopy(self.edges[2]))
        self.trajectory[dataType.edge_4].append(deepcopy(self.edges[3]))
        self.trajectory[dataType.target].append(deepcopy(self.target))
    
    # ---------------------------------------------------------
    # Functions that may to be overloaded in the child classes
    # ---------------------------------------------------------
    def _defineEdges(self):
        return
    
    def _calculateRadius(self):
        # Set minimal diameter to 0
        diameter = 0
        # Set radius such that all edges are enclosed
        for outerCount in range(0, len(self.edges)):
            for innerCount in range(outerCount+1, len(self.edges)):
                distanceToEdgeToEdge = np.linalg.norm(self.edges[outerCount] - self.edges[innerCount])
                if diameter < distanceToEdgeToEdge:
                    diameter = distanceToEdgeToEdge
        self.radius = 0.5 * diameter
    
    # --------------------------------
    # Calculate hitbox, colisions, etc
    # --------------------------------
    def _checkTarget(self):
        distanceToTarget = np.linalg.norm(self.target - self.center)
        if distanceToTarget < self.radius:
            self._log("Target position reached!")
            self.target = np.array([randint(-100,100),randint(-100,100)])
            self.trajectory[dataType.target].append(deepcopy(self.target))
            self.targetCounter += 1
    
    def _checkForCollision(self, shape):
        self._log("Collision with shape {}!".format(shape.shapeID))
        shape.stop()
        self.stop()
        return True
    
    def _checkForObstacles(self):
        self._listOfObstacles = []
        scanningRange = self.radius*5
        for shape in Shape.__LIST:
            if shape is self:
                continue
            distance = np.linalg.norm(self.center - shape.center)
            if distance < (self.radius + shape.radius):
                if self._checkForCollision(shape):
                    return
            elif distance < scanningRange:
                relativeVector = shape.center - self.center
                angle = (np.arctan2(relativeVector[1], relativeVector[0]) - np.arctan2(self._direction[1], self._direction[0]))
                # restrict angles to [-pi,+pi]
                while angle > np.pi:
                    angle -= 2*np.pi
                while angle < -np.pi:
                    angle += 2*np.pi
                # NOTE: for now we cheat by handing over the other objects's position and speed
                self._listOfObstacles.append(Obstacle(distance, angle, shape._velocity*shape._direction, shape.center, shape.radius))

