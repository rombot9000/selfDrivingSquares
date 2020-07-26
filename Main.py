#!/usr/bin/env python3

# Custom packages
from shapes import dataType, Shape, Rectangle
from drivers import Conditional
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
    global listOfRectangles, listOfDrivers
    for driver in listOfDrivers:
        driver.steer()
    Shape.moveAll(REACTION_TIME)

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
    listOfDrivers = []
    for i in range(0,numberOfRectangles):
        rectangle = Rectangle()
        rectangle.addToPlot(plot)
        listOfRectangles.append(rectangle)

        driver = Conditional()
        driver.setShape(rectangle)
        driver.addToPlot(plot)
        listOfDrivers.append(driver)

    animationInterval = int( 1000 * REACTION_TIME )
    anim = FuncAnimation(fig, updatePlot, None, interval=animationInterval)
    plt.show()
    
    # Raises error -> use show() for now
    #mywriter = FFMpegWriter(fps=60)
    #anim.save('trajectory.mp4',writer=mywriter)