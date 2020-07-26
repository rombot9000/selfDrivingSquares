from .Driver import Driver
import numpy as np

class Conditional(Driver):
    def __init__(self):
        super().__init__()
        self._performUTurn = False
    
    def steer(self):
        '''
        Calculate steering angle depending on object and target position
        '''
        if self._shape.isActive:
            self._checkTarget()
            if not self._adjustForObstacles():
                self._adjustForTarget()
            self._shape.setParams(self._steeringAngle, self._acceleration)

    def _projectSmallestDistance(self, direction, obstacle):
        '''
        Project the smallest distance to an obstacle given the current parameters
        '''
        # ignore obstacles behind driver
        if np.abs(obstacle.angle) > 3*np.pi/4:
            return np.inf, np.inf
        # Current distance vector
        d0 = self._shape._center - obstacle.center
        d0d0 = np.dot(d0,d0)
        # check if already in radius
        combinedRadii = self._shape._radius + obstacle.radius
        if d0d0 < combinedRadii*combinedRadii:
            return 0,0
        # Relative velocity dV
        dV = self._shape._velocity * direction - obstacle.velocity
        dVdV = np.dot(dV, dV)
        # Check if the relative velocity is finite
        if dVdV == 0:
            return np.inf, np.inf
        # Getting minimum of distance as a function of time yields the following:
        # t_min = dV*d0 / |dV|^2
        # Check if t_min > 0
        tMin = - np.dot(dV, d0) / dVdV
        if tMin < 0:
            return np.inf, tMin
        # Minumum distance
        dMin = np.linalg.norm(d0 + tMin * dV)
        # If collision is projected, get the collision time
        if dMin <= combinedRadii:
            tMin -= np.sqrt(tMin*tMin + (combinedRadii*combinedRadii - d0d0)/dVdV )
        return dMin, tMin
    
    def _adjustForObstacles(self):
        nextObstacle = None
        nextEvent    = 5
        rotationMatrix = np.array([[np.cos(self._shape._movingAngle), -np.sin(self._shape._movingAngle)],[np.sin(self._shape._movingAngle), np.cos(self._shape._movingAngle)]])
        steeringDirection = np.dot(rotationMatrix, self._shape._direction)
        for obstacle in self._shape._listOfObstacles:
            smallestDistance, projectedTime = self._projectSmallestDistance(steeringDirection, obstacle)
            if smallestDistance <= (self._shape._radius + obstacle.radius):
                if projectedTime < nextEvent:
                    nextObstacle = obstacle
                    nextEvent    = projectedTime
        
        if nextObstacle is None:
            return False

        print("Projected collision in {} seconds!".format(nextEvent))
        if abs(nextObstacle.angle) <= np.pi/2:
            if projectedTime < 1:
                #print("Max Break!")
                self._acceleration = -1
            elif projectedTime < 2:
                #print("Zero Acceleration!")
                self._acceleration = 0
        if obstacle.angle > 0:
            print("Steer left!")
            self._steeringAngle = -np.pi
        else:
            print("Steer right!")
            self._steeringAngle =  np.pi
        self._performUTurn = False
        return True
    
    def _adjustForTarget(self):
        '''
        Adjust parameters for target
        '''
        targetVector = self._target - self._shape._center
        theta = (np.arctan2(targetVector[1], targetVector[0]) - np.arctan2(self._shape._direction[1], self._shape._direction[0]))
        while theta > np.pi:
            theta -= 2*np.pi
        while theta < -np.pi:
            theta += 2*np.pi

        #check if rectangle has to do a u-turn
        if self._performUTurn and abs(theta) > self._shape._maxSteeringAngle:
            return
        elif abs(theta) > np.pi/2.0:
            self._performUTurn = True
        else:
            self._performUTurn = False
        
        self._steeringAngle = theta
        self._acceleration = 1