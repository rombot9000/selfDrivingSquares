from collections import namedtuple
import numpy as np
from random import randint
from enum import Enum
from copy import copy, deepcopy

class dataType(Enum):
    center         = 1
    edges          = 2
    steeringEdge   = 3
    _target         = 4
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
    def moveAll(timespan):
        '''
        Iterate move method for all shapes for given period
        '''
        t = 0.0
        activeShapes = [x for x in Shape.__LIST if x.isActive]
        while t < (timespan - 0.5*Shape._TIMESTEP):
            # move all shapes
            for shape in activeShapes:
                shape.move(Shape._TIMESTEP)
            # increase time
            t += Shape._TIMESTEP
        for shape in activeShapes:
            shape.updatePlotObjects()
    
    def __init__(self):
        #
        # Append instance of shape to list of shapes
        #
        Shape.__LIST.append(self)
        # public
        self.shapeID    = len(Shape.__LIST)
        self.isActive   = True
        # protected
        self._edges     = []
        self._radius    = 0
        self._center    = np.array([0,0])
        self._color     = '#{:06x}'.format(randint(0, 0xFFFFFF))
        self._trajectory = {}
        for key in dataType:
            self._trajectory[key] = []
        self._direction = np.array([randint(-100,100),randint(-100,100)])
        self._direction = self._direction / np.linalg.norm(self._direction)
        self._velocity  = 0
        self._listOfObstacles = []
    
        # call init funcs
        self._defineEdges()
        self._calculateRadius()
        self.__setCenter()
        self._defineEdges()
        self._addToTrajectory()
    
    def __setCenter(self):
        while len([x for x in Shape.__LIST if x is not self and self.doOverlapp(x)]) > 0:
            print("reset center")
            self._center = np.array([randint(-100,100),randint(-100,100)])
    
    def deactivate(self):
        '''
        deactivate shape and set inactive
        '''
        self.isActive = False
        self._velocity = 0.0
        self._color    = greyAsHex
        self.updateColor()
        self.updatePlotObjects()

    def updatePlotObjects(self):
        '''
        Is called to update all matplotlib plot objects
        Needs to be implemented in child class
        '''
        pass

    def updateColor(self):
        '''
        Needs to be implemented in child class
        '''
        pass
    
    def move(self):
        '''
        Is called to move shape
        Needs to be define in child class
        '''
        pass
    
    def _log(self, __VAR_ARGS__):
        '''
        Logger function
        '''
        print("SID {}: ".format(self.shapeID), __VAR_ARGS__)
    
    def _addToTrajectory(self):
        '''
        Update trajectory of shape
        '''
        self._trajectory[dataType.center].append(deepcopy(self._center))
        self._trajectory[dataType.edge_1].append(deepcopy(self._edges[0]))
        self._trajectory[dataType.edge_2].append(deepcopy(self._edges[1]))
        self._trajectory[dataType.edge_3].append(deepcopy(self._edges[2]))
        self._trajectory[dataType.edge_4].append(deepcopy(self._edges[3]))
    
    # ---------------------------------------------------------
    # Functions that may to be overloaded in the child classes
    # ---------------------------------------------------------
    def _defineEdges(self):
        '''
        Is called to define edges of the shape
        Needs to be implemented in child class
        '''
        pass
    
    def _calculateRadius(self):
        # Set minimal diameter to 0
        diameter = 0
        # Set radius such that all edges are enclosed
        for outerCount in range(0, len(self._edges)):
            for innerCount in range(outerCount+1, len(self._edges)):
                distanceToEdgeToEdge = np.linalg.norm(self._edges[outerCount] - self._edges[innerCount])
                if diameter < distanceToEdgeToEdge:
                    diameter = distanceToEdgeToEdge
        self._radius = 0.5 * diameter
    
    # --------------------------------
    # Calculate hitbox, colisions, etc
    # --------------------------------
    def isInRadius(self, position):
        return np.linalg.norm(self._center - position) < self._radius
    
    def doOverlapp(self, shape):
        return np.linalg.norm(self._center - shape._center) < (self._radius + shape._radius)
    
    def _checkForCollision(self, shape):
        self._log("Collision with shape {}!".format(shape.shapeID))
        return True
    
    def _checkForObstacles(self):
        self._listOfObstacles = []
        scanningRange = self._radius*5
        for shape in Shape.__LIST:
            # do not handle self
            if shape is self: continue
            
            # calc current distance
            distance = np.linalg.norm(self._center - shape._center)

            # check for colisions
            if distance < (self._radius + shape._radius):
                if self._checkForCollision(shape):
                    shape.deactivate()
                    self.deactivate()
                    return
            
            # set obstacle list
            if distance < scanningRange:
                relativeVector = shape._center - self._center
                angle = (np.arctan2(relativeVector[1], relativeVector[0]) - np.arctan2(self._direction[1], self._direction[0]))
                # restrict angles to [-pi,+pi]
                while angle > np.pi:
                    angle -= 2*np.pi
                while angle < -np.pi:
                    angle += 2*np.pi
                # NOTE: for now we cheat by handing over the other objects's position and speed
                self._listOfObstacles.append(Obstacle(distance, angle, shape._velocity*shape._direction, shape._center, shape._radius))
