import numpy as np
import random
import copy

class Rectangle:
    def __init__(self):
        self.width      = 1.0
        self.length     = 3.0
        self.radius     = 0.5*np.sqrt(self.width*self.width + self.length*self.length)
        self.center     = np.array([0.0, 0.0])
        self.trajectory = {}
        self.trajectory['center']         = []
        self.trajectory['edges']          = []
        self.trajectory['steeringEdge']   = []
        self.trajectory['targetPosition'] = []
        self.direction      = np.array([1.0, 0.0])
        self.steeringAngle  = 0.0
        self.edges          = [[None,None],[None,None]]
        self.calculateEdges()
        self.velocity       = 1
        self.dt             = 0.5
        self.targetPosition = np.array([random.randint(-100,100),random.randint(-100,100)])
        self.targetCounter  = 0
        #
        #
        #
        self.trajectory['center'].append(copy.deepcopy(self.center))
        self.trajectory['edges'].append(copy.deepcopy(self.edges))
        self.trajectory['targetPosition'].append(copy.deepcopy(self.targetPosition))
    
    # -------------------------
    # Main task to for movement
    # -------------------------
    def run(self, timeSpan):
        #self.calculateEdges()
        #print(self.edges[0][0], self.edges[0][1])
        #print(self.edges[1][0], self.edges[1][1])
        self.steer()
        t = 0.0
        while t < (timeSpan - 0.5*self.dt):
            self.moveEdges(self.dt)
            self.calculateCenter()
            self.checkTarget()
            t += self.dt
        if not np.isnan(self.center[0]):
            self.trajectory['center'].append(self.center)
            self.trajectory['edges'].append(copy.deepcopy(self.edges))
    
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
        self.edges[0][0] = self.center + 0.5*self.length*self.direction - 0.5*self.width*orthogonalDirectionLR
        self.edges[0][1] = self.edges[0][0] + self.width*orthogonalDirectionLR
        self.edges[1][0] = self.edges[0][0] - self.length*self.direction
        self.edges[1][1] = self.edges[1][0] + self.width*orthogonalDirectionLR
    
    # ----------------------------------------------------------------
    # Calculate steering angle depending on object and target position
    # ----------------------------------------------------------------
    def steer(self):
        targetVector = self.targetPosition - self.center
        theta = (np.arctan2(targetVector[1], targetVector[0]) - np.arctan2(self.direction[1], self.direction[0]))
        #print(theta)
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
            if not np.isnan(self.edges[0][0][0]):
                self.trajectory['steeringEdge'].append(copy.copy(self.edges[0][0]))
        else:
            self.calculateEdgeRotation(timeSpan, self.edges[0][1], self.edges[1][1])
            ##Calculate remaining edges
            self.direction   = (self.edges[0][1] - self.edges[1][1])/self.length
            orthogonalDirectionRL = np.array([-self.direction[1], self.direction[0]])
            self.edges[0][0] = self.edges[0][1] + self.width*orthogonalDirectionRL
            self.edges[1][0] = self.edges[1][1] + self.width*orthogonalDirectionRL
            if not np.isnan(self.edges[0][1][0]):
                self.trajectory['steeringEdge'].append(copy.copy(self.edges[0][1]))
    
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
    
    # --------------------------------
    # Calculate hitbox, colisions, etc
    # --------------------------------
    def checkTarget(self):
        distanceToTarget = np.linalg.norm(self.targetPosition - self.center)
        if distanceToTarget < self.radius:
            print("Target position reached!\n")
            self.targetPosition = np.array([random.randint(-100,100),random.randint(-100,100)])
            self.trajectory['targetPosition'].append(copy.deepcopy(self.targetPosition))
            self.targetCounter += 1
