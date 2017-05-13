#!/opt/local/bin/python3.5

# Custom packages
from selfDriving.Shape import dataType, Shape
from selfDriving.Rectangle import Rectangle
# Numerics and plotting
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from matplotlib.animation import FuncAnimation, FFMpegWriter
# Parse cmd line options
import argparse

# ----------------
# Global variables
# ----------------
reactionTime = 0.1

def updatePlot(i):
    global listOfRectangles
    for rectangle in listOfRectangles:
        rectangle.steer()
    Shape.moveAll(reactionTime)
    for j in range(0, numberOfRectangles):
        rectangle = listOfRectangles[j]
        plgn[j].set_xy([rectangle.edges[0][0], rectangle.edges[0][1], rectangle.edges[1][1], rectangle.edges[1][0]])
        circ[j].center = rectangle.center[0], rectangle.center[1] 
        plts[j][0].set_data(*zip(*rectangle.trajectory[dataType.center]))
        if rectangle.isMoving:
            trgt[j].set_offsets([rectangle.target[0], rectangle.target[1]])

def setupPlot(rectangle):
    plgn.append(ax.add_patch(Polygon([rectangle.edges[0][0], rectangle.edges[0][1], rectangle.edges[1][1], rectangle.edges[1][0]], closed=True, fill=True, color=rectangle.color)))
    circ.append(ax.add_patch(Circle(rectangle.center, rectangle.radius, fill=False, color=rectangle.color, linestyle='dotted')))
    plts.append(ax.plot(*zip(*rectangle.trajectory[dataType.center]), color=rectangle.color, linestyle='dotted', alpha=0.25))
    trgt.append(ax.scatter(rectangle.target[0], rectangle.target[1], color=rectangle.color, marker='x'))  

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
    ax  = fig.add_subplot(111)
    ax.set_aspect(1)
    ax.axis([-150,150,-150,150])
    ax.set_axis_off()
    plgn = []
    circ = []
    plts = []
    trgt = [] 
    listOfRectangles = []
    for i in range(0,numberOfRectangles):
        listOfRectangles.append(Rectangle())
        setupPlot(listOfRectangles[-1])
    animationInterval = int( 1000 * reactionTime )
    anim = FuncAnimation(fig, updatePlot, None, interval=animationInterval)
    plt.show()
    
    # Raises error -> use show() for now
    #mywriter = FFMpegWriter(fps=60)
    #anim.save('trajectory.mp4',writer=mywriter)