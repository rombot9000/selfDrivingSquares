#!/usr/bin/env python3

# Custom packages
from shapes import dataType, Shape, Rectangle
# Numerics and plotting
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
# Parse cmd line options
import argparse

# ----------------
# Global variables
# ----------------
REACTION_TIME = 0.1

def updatePlot(i):
    global listOfRectangles
    for rectangle in listOfRectangles:
        rectangle.steer()
    Shape.moveAll(REACTION_TIME)
    for rectangle in listOfRectangles:
        if rectangle.isActive:
            rectangle.updatePlotObjects()

if __name__ == '__main__':
    # --------------------
    # Command line options
    # --------------------
    parser = argparse.ArgumentParser(description='Self driving shapes')
    parser.add_argument('-r', type=int, default=2, help='The number of rectangles to simulate')
    args = parser.parse_args()
    numberOfRectangles = args.r
    # ----------------------------
    # Show animation of trajectory
    # ----------------------------
    fig = plt.figure()
    plot  = fig.add_subplot(111)
    plot.set_aspect(1)
    plot.axis([-150,150,-150,150])
    plot.set_axis_off()

    listOfRectangles = []
    for i in range(0,numberOfRectangles):
        rectangle = Rectangle()
        rectangle.addToPlot(plot)
        listOfRectangles.append(rectangle)

    animationInterval = int( 1000 * REACTION_TIME )
    anim = FuncAnimation(fig, updatePlot, None, interval=animationInterval)
    plt.show()
    
    # Raises error -> use show() for now
    #mywriter = FFMpegWriter(fps=60)
    #anim.save('trajectory.mp4',writer=mywriter)