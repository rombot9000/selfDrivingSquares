from .Shape import Shape, dataType
import numpy as np
from copy import copy, deepcopy

class Rectangle(Shape):
    def __init__(self):
        self._width      = 12.0
        self._length     = 36.0
        Shape.__init__(self)
        self._maxSteeringAngle = np.pi/4
        self._maxAcceleration = 5
        self._maxBreak        = 10
        self._acceleration    = self._maxAcceleration
        self._airResistance   = 0.04
        self._breakResistance = 0.0
        self._steeringAngle   = 0.0
        self._performUTurn    = False
    
    # -------------------------
    # Main task to for movement
    # -------------------------
    def move(self):
        self._accelerate(Shape._TIMESTEP)
        self._moveEdges(Shape._TIMESTEP)
        self._calculateCenter()
        self._checkTarget()
        self._checkForObstacles()
        self._addToTrajectory()

    def setParams(self):
        pass
    # ------------------------------------
    # Overload functions from parent class
    # ------------------------------------
    def _defineEdges(self):
        # 0---1
        # |   |
        # |   |
        # |   |
        # 3---2
        orthogonalDirectionLR = np.array([self._direction[1], -self._direction[0]])
        self.edges = [None, None,None, None]
        self.edges[0] = self.center + 0.5*self._length*self._direction - 0.5*self._width*orthogonalDirectionLR
        self.edges[1] = self.edges[0] + self._width*orthogonalDirectionLR
        self.edges[3] = self.edges[0] - self._length*self._direction
        self.edges[2] = self.edges[3] + self._width*orthogonalDirectionLR
    
    def _calculateRadius(self):
        self.radius = 0.5*np.sqrt(self._width*self._width + self._length*self._length)
    
    # ------------------------------------
    # Auxilliary functions for calculating
    # position, direction, etc of object
    # ------------------------------------
    def _calculateCenter(self):
        self.center = 0.25*(self.edges[0] + self.edges[1] + self.edges[3] + self.edges[2])
    
    def _calculateDirection(self):
        self._direction = 0.5*(self.edges[0] + self.edges[1] - self.edges[3] - self.edges[2])/self._length
    
    # ----------------------------------------------------------------
    # Calculate steering angle depending on object and target position
    # ----------------------------------------------------------------
    def steer(self):
        if not self.isMoving:
            return
        targetVector = self.target - self.center
        theta = (np.arctan2(targetVector[1], targetVector[0]) - np.arctan2(self._direction[1], self._direction[0]))
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi
        #check if rectangle has to do a u-turn
        if self._performUTurn and abs(theta) > self._maxSteeringAngle:
            self._adjustAngleForObstacles()
            return
        elif abs(theta) > np.pi/2.0:
            self._performUTurn = True
        else:
            self._performUTurn = False
        #restrict angles to [+30,-30]
        if theta > self._maxSteeringAngle:
            theta = self._maxSteeringAngle
        elif theta < -self._maxSteeringAngle:
            theta = -self._maxSteeringAngle
        self._steeringAngle = theta
        self._acceleration    = self._maxAcceleration
        self._adjustAngleForObstacles()
    
    def _adjustAngleForObstacles(self):
        nextObstacle = None
        nextEvent    = 5
        rotationMatrix = np.array([[np.cos(self._steeringAngle), -np.sin(self._steeringAngle)],[np.sin(self._steeringAngle), np.cos(self._steeringAngle)]])
        steeringDirection = np.dot(rotationMatrix, self._direction)
        for obstacle in self._listOfObstacles:
            smallestDistance, projectedTime = self._projectSmallestDistance(steeringDirection, obstacle)
            if smallestDistance <= (self.radius + obstacle.radius):
                if projectedTime < nextEvent:
                    nextObstacle = obstacle
                    nextEvent    = projectedTime
        if nextObstacle is not None:
            self._log("Projected collision in {} seconds!".format(nextEvent))
            #if abs(nextObstacle.angle) <= np.pi/2:
                #if obstacle.distance < 1.5*(self.radius + obstacle.radius):
                    #self._log("Max Break!")
                    #self._acceleration = -self._maxBreak
                #elif obstacle.distance < 2*(self.radius + obstacle.radius):
                    #self._log("Zero Acceleration!")
                    #self._acceleration = 0
            if obstacle.angle > 0:
                self._log("Steer left!")
                self._steeringAngle = -self._maxSteeringAngle
            else:
                self._log("Steer right!")
                self._steeringAngle =  self._maxSteeringAngle
            self._performUTurn = False
    
    def _projectSmallestDistance(self, direction, obstacle):
        # Relative velocity dV
        dV = self._velocity * direction - obstacle.velocity
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
        # Minumum distance
        dMin = np.linalg.norm(d0 + tMin * dV)
        #If collision is projected, get the collision time
        combinedRadii = self.radius + obstacle.radius
        if dMin <= combinedRadii:
            d0d0 = np.dot(d0,d0)
            tMin -= np.sqrt(tMin*tMin + (combinedRadii*combinedRadii - d0d0)/dVdV )
        return dMin, tMin
    
    #
    # Check if one of the edges of the input shape is within self
    #
    def _checkForCollision(self, shape):
        #
        # Prepare constants for Collision detection
        #
        l = self.edges[0] - self.edges[3]
        w = self.edges[2] - self.edges[3]
        l2l1        = l[1]/l[0]
        denominator = w[1] - w[0] * l2l1
        for edge in shape.edges:
            if np.linalg.norm(edge - self.center) > self.radius:
                continue
            x = edge - self.edges[3]
            s = (x[1] - x[0] * l2l1)/denominator
            if (s>1) or (s<0):
                continue
            r = x[0] - s*w[0]
            if r>l[0] or r<0:
                continue
            self._log("Collision with shape {}!".format(shape.shapeID))
            self.stop()
            return True
        return False

    # ------------------------------------------------------------
    # Calculate movement of object for given timeSpan
    # depending on steeringAngle, current velocity, direction, etc
    # ------------------------------------------------------------
    def _accelerate(self, timeStep):
        # v(t + dt) = v(t) + a*dt - c*v(t)^2*dt
        self._velocity += (self._acceleration - self._airResistance*self._velocity*self._velocity)*timeStep
    
    def _moveEdges(self, timeSpan):
        if self._steeringAngle > 0:
            self._calculateEdgeRotation(timeSpan, self.edges[0], self.edges[3])
            #Calculate remaining edges
            self._direction   = (self.edges[0] - self.edges[3])/self._length
            orthogonalDirectionLR = np.array([self._direction[1], -self._direction[0]])
            self.edges[1] = self.edges[0] + self._width*orthogonalDirectionLR
            self.edges[2] = self.edges[3] + self._width*orthogonalDirectionLR
        else:
            self._calculateEdgeRotation(timeSpan, self.edges[1], self.edges[2])
            ##Calculate remaining edges
            self._direction   = (self.edges[1] - self.edges[2])/self._length
            orthogonalDirectionRL = np.array([-self._direction[1], self._direction[0]])
            self.edges[0] = self.edges[1] + self._width*orthogonalDirectionRL
            self.edges[3] = self.edges[2] + self._width*orthogonalDirectionRL
            
    
    def _calculateEdgeRotation(self, timeSpan, frontEdge, backEdge):
        backEdge += self._velocity*timeSpan*self._direction
        rotationMatrix = np.array([[np.cos(self._steeringAngle), -np.sin(self._steeringAngle)],[np.sin(self._steeringAngle), np.cos(self._steeringAngle)]])
        steeringDirection = np.dot(rotationMatrix, self._direction)
        p = np.dot(steeringDirection, frontEdge - backEdge) / np.dot(steeringDirection,steeringDirection)
        q = (self._velocity*timeSpan - 2.0*self._length) * self._velocity*timeSpan / np.dot(steeringDirection,steeringDirection)
        sqrtTerm = np.sqrt(p*p - q)
        if sqrtTerm < 0:
            print(sqrtTerm)
            sqrtTerm = 0.0
        alpha = - p + sqrtTerm
        frontEdge += alpha * steeringDirection
    
    