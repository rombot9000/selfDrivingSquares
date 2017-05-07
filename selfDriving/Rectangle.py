from .Shape import Shape, dataType
import numpy as np
from copy import copy, deepcopy

class Rectangle(Shape):
    def __init__(self):
        self.width      = 6.0
        self.length     = 18.0
        self.radius     = 0.5*np.sqrt(self.width*self.width + self.length*self.length)
        Shape.__init__(self)
        self.maxSteeringAngle = np.pi/4
        self.maxAcceleration = 5
        self.maxBreak        = 10
        self.acceleration    = self.maxAcceleration
        self.airResistance   = 0.04
        self.breakResistance = 0.0
        self.steeringAngle   = 0.0
        self.performUTurn    = False
        self.targetCounter   = 0
    
    # -------------------------
    # Main task to for movement
    # -------------------------
    def move(self):
        if not self.isMoving:
            return
        self.accelerate(self.timeStep)
        self.moveEdges(self.timeStep)
        self.calculateCenter()
        self.checkTarget()
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
        if not self.isMoving:
            return
        targetVector = self.target - self.center
        theta = (np.arctan2(targetVector[1], targetVector[0]) - np.arctan2(self.direction[1], self.direction[0]))
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi
        #check if rectangle has to do a u-turn
        if self.performUTurn and abs(theta) > self.maxSteeringAngle:
            self.adjustAngleForObstacles()
            return
        elif abs(theta) > np.pi/2.0:
            self.performUTurn = True
        else:
            self.performUTurn = False
        #restrict angles to [+30,-30]
        if theta > self.maxSteeringAngle:
            theta = self.maxSteeringAngle
        elif theta < -self.maxSteeringAngle:
            theta = -self.maxSteeringAngle
        self.steeringAngle = theta
        self.acceleration    = self.maxAcceleration
        self.adjustAngleForObstacles()
    
    def adjustAngleForObstacles(self):
        nextObstacle = None
        nextEvent    = 5
        rotationMatrix = np.array([[np.cos(self.steeringAngle), -np.sin(self.steeringAngle)],[np.sin(self.steeringAngle), np.cos(self.steeringAngle)]])
        steeringDirection = np.dot(rotationMatrix, self.direction)
        for obstacle in self.listOfObstacles:
            smallestDistance, projectedTime = self.projectSmallestDistance(steeringDirection, obstacle)
            if smallestDistance <= (self.radius + obstacle.radius):
                if projectedTime < nextEvent:
                    nextObstacle = obstacle
                    nextEvent    = projectedTime
        if nextObstacle is not None:
            print('SID {}: Projected collision in {} seconds!'.format(self.shapeID, nextEvent))
            if obstacle.distance < 1.5*(self.radius + obstacle.radius):
                print("SID {}: Max Break!".format(self.shapeID))
                self.acceleration = -self.maxBreak
            elif obstacle.distance < 2*(self.radius + obstacle.radius):
                self.acceleration = 0
            if obstacle.angle > 0:
                self.steeringAngle = -self.maxSteeringAngle
            else:
                self.steeringAngle =  self.maxSteeringAngle
            self.performUTurn = False
    
    def projectSmallestDistance(self, direction, obstacle):
        # Relative velocity dV
        dV = self.velocity * direction - obstacle.velocity
        dVdV = np.dot(dV, dV)
        # Check if the relative velocity is finite
        if dVdV == 0:
            return np.inf, np.inf
        # Current distance vector
        d0 = self.center - obstacle.center
        # Getting minimum of distance as a function of time yields the following:
        # t_min = dV*d0 / |dV|^2
        # Check if t_min > 0
        tMin = - np.dot(dV, d0) / dVdV
        if tMin < 0:
            return np.inf, tMin
        # Return minumum distance
        dMin = d0 + tMin * dV
        return np.linalg.norm(dMin), tMin

    # ------------------------------------------------------------
    # Calculate movement of object for given timeSpan
    # depending on steeringAngle, current velocity, direction, etc
    # ------------------------------------------------------------
    def accelerate(self, timeStep):
        # v(t + dt) = v(t) + a*dt - c*v(t)^2*dt
        self.velocity += (self.acceleration - self.airResistance*self.velocity*self.velocity)*timeStep
    
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
        rotationMatrix = np.array([[np.cos(self.steeringAngle), -np.sin(self.steeringAngle)],[np.sin(self.steeringAngle), np.cos(self.steeringAngle)]])
        steeringDirection = np.dot(rotationMatrix, self.direction)
        p = np.dot(steeringDirection, frontEdge - backEdge) / np.dot(steeringDirection,steeringDirection)
        q = (self.velocity*timeSpan - 2.0*self.length) * self.velocity*timeSpan / np.dot(steeringDirection,steeringDirection)
        sqrtTerm = np.sqrt(p*p - q)
        if sqrtTerm < 0:
            print(sqrtTerm)
            sqrtTerm = 0.0
        alpha = - p + sqrtTerm
        frontEdge += alpha * steeringDirection
    
    