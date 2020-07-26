from .Shape import Shape, dataType
import numpy as np
from copy import copy, deepcopy
from matplotlib.patches import Polygon, Circle

class Rectangle(Shape):
    def __init__(self):
        # set shape specific members before calling shape init
        self._width      = 12.0
        self._length     = 36.0
        Shape.__init__(self)
        # set movement specific members
        self._maxSteeringAngle = np.pi/3
        self._maxAcceleration = 5
        self._maxBreak        = 10
        self._acceleration    = self._maxAcceleration
        self._airResistance   = 0.04
        self._breakResistance = 0.0
        self._steeringAngle   = 0.0
        self._movingAngle     = 0.0
        self._performUTurn    = False
    
    def move(self, timestep : int):
        '''
        Move shape by timestep
        '''
        self._accelerate(timestep)
        self._rotateMovingAngle()
        self._moveEdges(timestep)
        self._calculateCenter()
        self._checkTarget()
        self._checkForObstacles()
        self._addToTrajectory()

    def setParams(self, steeringAngle, acceleration):
        '''
        Set movement params
        '''
        self._steeringAngle = steeringAngle
        self._acceleration = acceleration

    def addToPlot(self, plot):
        '''
        Add self to plot
        '''
        self._circ = plot.add_patch(Circle(self._center, self._radius, fill=False, color=self._color, linestyle='dotted', zorder=1))
        self._plgn = plot.add_patch(Polygon(self._edges, closed=True, fill=True, color=self._color, zorder=2))
        self._trgt = plot.scatter(self._target[0], self._target[1], color=self._color, marker='x', zorder=2)
        
        self._traj = []
        self._traj.append(plot.plot(*zip(*self._trajectory[dataType.center]), color=self._color, linestyle='dotted', alpha=0.25, zorder=1))
        #self.plts.append(plot.plot(*zip(*self._trajectory[dataType.edge_1]),
                            #*zip(*self._trajectory[dataType.edge_2]),
                            #*zip(*self._trajectory[dataType.edge_3]),
                            #*zip(*self._trajectory[dataType.edge_4]),
                            #color=self._color, linestyle='dotted', alpha=0.25, zorder=1))


    def updatePlotObjects(self):
        '''
        update plot objects
        '''
        self._plgn.set_xy(self._edges)
        self._circ.center = self._center[0], self._center[1] 
        self._trgt.set_offsets([self._target[0], self._target[1]])
        
        self._traj[0][0].set_data(*zip(*self._trajectory[dataType.center]))
        #self._traj[0].set_data(*zip(*self._trajectory[dataType.edge_1]))
        #self._traj[1].set_data(*zip(*self._trajectory[dataType.edge_2]))
        #self._traj[2].set_data(*zip(*self._trajectory[dataType.edge_3]))
        #self._traj[3].set_data(*zip(*self._trajectory[dataType.edge_4]))    

    
    def _defineEdges(self):
        '''
        Define rectangular shape
        0---1
        |   |
        |   |
        |   |
        3---2
        '''
        orthogonalDirectionLR = np.array([self._direction[1], -self._direction[0]])
        self._edges = [None, None,None, None]
        self._edges[0] = self._center + 0.5*self._length*self._direction - 0.5*self._width*orthogonalDirectionLR
        self._edges[1] = self._edges[0] + self._width*orthogonalDirectionLR
        self._edges[3] = self._edges[0] - self._length*self._direction
        self._edges[2] = self._edges[3] + self._width*orthogonalDirectionLR
    
    def _calculateRadius(self):
        self._radius = 0.5*np.sqrt(self._width*self._width + self._length*self._length)
    
    # ------------------------------------
    # Auxilliary functions for calculating
    # position, direction, etc of object
    # ------------------------------------
    def _calculateCenter(self):
        self._center = 0.25*(self._edges[0] + self._edges[1] + self._edges[3] + self._edges[2])
    
    def _calculateDirection(self):
        self._direction = 0.5*(self._edges[0] + self._edges[1] - self._edges[3] - self._edges[2])/self._length
    
    def steer(self):
        '''
        Calculate steering angle depending on object and target position
        '''
        if not self.isActive:
            return
        targetVector = self._target - self._center
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
        rotationMatrix = np.array([[np.cos(self._movingAngle), -np.sin(self._movingAngle)],[np.sin(self._movingAngle), np.cos(self._movingAngle)]])
        steeringDirection = np.dot(rotationMatrix, self._direction)
        for obstacle in self._listOfObstacles:
            smallestDistance, projectedTime = self._projectSmallestDistance(steeringDirection, obstacle)
            if smallestDistance <= (self._radius + obstacle.radius):
                if projectedTime < nextEvent:
                    nextObstacle = obstacle
                    nextEvent    = projectedTime
        if nextObstacle is not None:
            self._log("Projected collision in {} seconds!".format(nextEvent))
            #if abs(nextObstacle.angle) <= np.pi/2:
                #if obstacle.distance < 1.5*(self._radius + obstacle.radius):
                    #self._log("Max Break!")
                    #self._acceleration = -self._maxBreak
                #elif obstacle.distance < 2*(self._radius + obstacle.radius):
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
        d0 = self._center - obstacle.center
        # Getting minimum of distance as a function of time yields the following:
        # t_min = dV*d0 / |dV|^2
        # Check if t_min > 0
        tMin = - np.dot(dV, d0) / dVdV
        if tMin < 0:
            return np.inf, tMin
        # Minumum distance
        dMin = np.linalg.norm(d0 + tMin * dV)
        #If collision is projected, get the collision time
        combinedRadii = self._radius + obstacle.radius
        if dMin <= combinedRadii:
            d0d0 = np.dot(d0,d0)
            tMin -= np.sqrt(tMin*tMin + (combinedRadii*combinedRadii - d0d0)/dVdV )
        return dMin, tMin
    
    def _checkForCollision(self, shape):
        '''
        Check if one of the edges of the input shape is within self
        '''
        # Prepare constants for Collision detection
        l = self._edges[0] - self._edges[3]
        w = self._edges[2] - self._edges[3]
        l2l1        = l[1]/l[0]
        denominator = w[1] - w[0] * l2l1
        # check all edges
        for edge in shape._edges:
            if np.linalg.norm(edge - self._center) > self._radius:
                continue
            x = edge - self._edges[3]
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

    def _accelerate(self, timeStep):
        '''
        Calculate movement of object for given timespan
        depending on steeringAngle, current velocity, direction, etc
        v(t + dt) = v(t) + a*dt - c*v(t)^2*dt
        '''
        self._velocity += (self._acceleration - self._airResistance*self._velocity*self._velocity)*timeStep

    def _rotateMovingAngle(self):
        if self._movingAngle > self._steeringAngle:
            self._movingAngle -= np.pi/10
        elif self._movingAngle < self._steeringAngle:
            self._movingAngle += np.pi/10
    
    def _moveEdges(self, timespan):
        if self._movingAngle > 0:
            self._calculateEdgeRotation(timespan, self._edges[0], self._edges[3])
            #Calculate remaining edges
            self._direction   = (self._edges[0] - self._edges[3])/self._length
            orthogonalDirectionLR = np.array([self._direction[1], -self._direction[0]])
            self._edges[1] = self._edges[0] + self._width*orthogonalDirectionLR
            self._edges[2] = self._edges[3] + self._width*orthogonalDirectionLR
        else:
            self._calculateEdgeRotation(timespan, self._edges[1], self._edges[2])
            ##Calculate remaining edges
            self._direction   = (self._edges[1] - self._edges[2])/self._length
            orthogonalDirectionRL = np.array([-self._direction[1], self._direction[0]])
            self._edges[0] = self._edges[1] + self._width*orthogonalDirectionRL
            self._edges[3] = self._edges[2] + self._width*orthogonalDirectionRL
            
    
    def _calculateEdgeRotation(self, timespan, frontEdge, backEdge):
        backEdge += self._velocity*timespan*self._direction
        rotationMatrix = np.array([[np.cos(self._movingAngle), -np.sin(self._movingAngle)],[np.sin(self._movingAngle), np.cos(self._movingAngle)]])
        steeringDirection = np.dot(rotationMatrix, self._direction)
        p = np.dot(steeringDirection, frontEdge - backEdge) / np.dot(steeringDirection,steeringDirection)
        q = (self._velocity*timespan - 2.0*self._length) * self._velocity*timespan / np.dot(steeringDirection,steeringDirection)
        sqrtTerm = np.sqrt(p*p - q)
        if sqrtTerm < 0:
            print(sqrtTerm)
            sqrtTerm = 0.0
        alpha = - p + sqrtTerm
        frontEdge += alpha * steeringDirection
    
    