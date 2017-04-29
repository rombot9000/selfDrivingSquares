from .Shape import Shape, dataType
import numpy as np
from random import randint
from copy import copy, deepcopy

class Rectangle(Shape):
    def __init__(self):
        self.width      = 3.0
        self.length     = 9.0
        self.radius     = 0.5*np.sqrt(self.width*self.width + self.length*self.length)
        Shape.__init__(self)
        self.steeringAngle  = 0.0
        self.performUTurn   = False
        self.velocity       = randint(4,7)
        self.targetCounter  = 0
    
    # -------------------------
    # Main task to for movement
    # -------------------------
    def run(self, timeSpan):
        if self.isMoving:
            self.steer()
            t = 0.0
            while t < (timeSpan - 0.5*self.timeStep):
                self.moveEdges(self.timeStep)
                self.calculateCenter()
                self.checkTarget()
                t += self.timeStep
                self.checkForObstacles()
        self.addToTrajectory()
    
    # ------------------------------------
    # Auxilliary functions for calculating
    # position, direction, etc of object
    # ------------------------------------
    def calculateCenter(self):
        self.center = 0.25*(self.edges[0][0] + self.edges[0][1] + self.edges[1][0] + self.edges[1][1])
    
    def calculateDirection(self):
        self.direction = 0.5*(self.edges[0][0] + self.edges[0][1] - self.edges[1][0] - self.edges[1][1])/self.length
    
    def calculateEdges(self):
        # 00---01
        # |     |
        # |     |
        # |     |
        # 10---11
        orthogonalDirectionLR = np.array([self.direction[1], -self.direction[0]])
        self.edges = [[None, None], [None, None]]
        self.edges[0][0] = self.center + 0.5*self.length*self.direction - 0.5*self.width*orthogonalDirectionLR
        self.edges[0][1] = self.edges[0][0] + self.width*orthogonalDirectionLR
        self.edges[1][0] = self.edges[0][0] - self.length*self.direction
        self.edges[1][1] = self.edges[1][0] + self.width*orthogonalDirectionLR
    
    # ----------------------------------------------------------------
    # Calculate steering angle depending on object and target position
    # ----------------------------------------------------------------
    def steer(self):
        targetVector = self.target - self.center
        theta = (np.arctan2(targetVector[1], targetVector[0]) - np.arctan2(self.direction[1], self.direction[0]))
        #check if rectangle has to do a u-turn
        if self.performUTurn and abs(theta) > np.pi/6.0:
            return
        elif abs(theta) > np.pi/2.0:
            self.performUTurn = True
        else:
            self.performUTurn = False
        #restrict angles to [+30,-30]
        if theta > np.pi/6.0:
            theta = np.pi/6.0
        elif theta < -np.pi/6.0:
            theta = -np.pi/6.0
        self.steeringAngle = theta
        self.rotationMatrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    
    # ------------------------------------------------------------
    # Calculate movement of object for given timeSpan
    # depending on steeringAngle, current velocity, direction, etc
    # ------------------------------------------------------------
    def moveEdges(self, timeSpan):
        if self.steeringAngle > 0:
            self.calculateEdgeRotation(timeSpan, self.edges[0][0], self.edges[1][0])
            #Calculate remaining edges
            self.direction   = (self.edges[0][0] - self.edges[1][0])/self.length
            orthogonalDirectionLR = np.array([self.direction[1], -self.direction[0]])
            self.edges[0][1] = self.edges[0][0] + self.width*orthogonalDirectionLR
            self.edges[1][1] = self.edges[1][0] + self.width*orthogonalDirectionLR
        else:
            self.calculateEdgeRotation(timeSpan, self.edges[0][1], self.edges[1][1])
            ##Calculate remaining edges
            self.direction   = (self.edges[0][1] - self.edges[1][1])/self.length
            orthogonalDirectionRL = np.array([-self.direction[1], self.direction[0]])
            self.edges[0][0] = self.edges[0][1] + self.width*orthogonalDirectionRL
            self.edges[1][0] = self.edges[1][1] + self.width*orthogonalDirectionRL
            
    
    def calculateEdgeRotation(self, timeSpan, frontEdge, backEdge):
        backEdge += self.velocity*timeSpan*self.direction
        steeringDirection = np.dot(self.rotationMatrix, self.direction)
        p = np.dot(steeringDirection, frontEdge - backEdge) / np.dot(steeringDirection,steeringDirection)
        q = (self.velocity*timeSpan - 2.0*self.length) * self.velocity*timeSpan / np.dot(steeringDirection,steeringDirection)
        sqrtTerm = np.sqrt(p*p - q)
        if sqrtTerm < 0:
            print(sqrtTerm)
            sqrtTerm = 0.0
        alpha = - p + sqrtTerm
        frontEdge += alpha * steeringDirection
    
    # --------------
    # Evasive action
    # --------------
    
