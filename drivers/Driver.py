import numpy as np
from random import randint

class Driver:
    def __init__(self):
        '''
        Base class for driver logic
        '''
        self._steeringAngle = 0
        self._acceleration = 0
        self._targetCounter = 0
        self._target = np.array([randint(-100,100),randint(-100,100)])
    
    def setShape(self, shape):
        '''
        Set shape used to drive
        '''
        self._shape = shape
        self._color = shape._color

    def steer(self):
        '''
        Needs to be implemented in child class
        '''
        pass

    def addToPlot(self, plot):
        '''
        Add self to plot
        '''
        if self._target is None:
            self._target = np.array([randint(-100,100),randint(-100,100)])
        self._targetPlot = plot.scatter(self._target[0], self._target[1], color=self._color, marker='x', zorder=2)
    
    def _checkTarget(self):
        if self._shape.isInRadius(self._target):
            print("Target position reached!")
            self._updateTarget()

    def _updateTarget(self):
        '''
        '''
        self._target = np.array([randint(-100,100),randint(-100,100)])
        self._targetPlot.set_offsets([self._target[0], self._target[1]])