from .Shape import Shape, dataType
import numpy as np
from copy import copy, deepcopy
from matplotlib.patches import Polygon, Circle

class Rectangle(Shape):
    def __init__(self):
        # set shape specific members before calling shape init
        self._width      = 12.0
        self._length     = 36.0
        super().__init__()
        # set movement specific members
        self._maxSteeringAngle = np.pi/3
        self._maxAcceleration = 5
        self._maxBreak        = 10
        self._acceleration    = self._maxAcceleration
        self._airResistance   = 0.04
        self._breakResistance = 0.0
        self._steeringAngle   = 0.0
        self._movingAngle     = 0.0
    
    def move(self, timestep : int):
        '''
        Move shape by timestep
        '''
        self._rotateMovingAngle()
        self._accelerate(timestep)
        self._moveEdges(timestep)
        self._calculateCenter()
        self._addToTrajectory()
        self._checkForObstacles()
    
    def deactivate(self):
        self._acceleration = 0
        super().deactivate()

    def setParams(self, steeringAngle, acceleration):
        '''
        Set movement params
        '''
        # restric and set angle
        if steeringAngle > self._maxSteeringAngle:
            steeringAngle = self._maxSteeringAngle
        elif steeringAngle < -self._maxSteeringAngle:
            steeringAngle = -self._maxSteeringAngle
        self._steeringAngle = steeringAngle

        # restrict and set acceleration
        if acceleration > 1:
            acceleration = 1
        if acceleration < -1:
            acceleration = -1
        self._acceleration = acceleration * self._maxAcceleration

    def addToPlot(self, plot):
        '''
        Add self to plot
        '''
        self._circ = plot.add_patch(Circle(self._center, self._radius, fill=False, color=self._color, linestyle='dotted', zorder=1))
        self._plgn = plot.add_patch(Polygon(self._edges, closed=True, fill=True, color=self._color, zorder=2))
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
        self._traj[0][0].set_data(*zip(*self._trajectory[dataType.center]))
        #self._traj[0].set_data(*zip(*self._trajectory[dataType.edge_1]))
        #self._traj[1].set_data(*zip(*self._trajectory[dataType.edge_2]))
        #self._traj[2].set_data(*zip(*self._trajectory[dataType.edge_3]))
        #self._traj[3].set_data(*zip(*self._trajectory[dataType.edge_4]))

    def updateColor(self):
        self._plgn.set_color(self._color)

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
            return True
        return False

    def _accelerate(self, timestep):
        '''
        Calculate movement of object for given timespan
        depending on steeringAngle, current velocity, direction, etc
        v(t + dt) = v(t) + a*dt - c*v(t)^2*dt
        '''
        self._velocity += (self._acceleration - self._airResistance*self._velocity*self._velocity)*timestep
        if self._velocity < 0: self._velocity = 0

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
    
    