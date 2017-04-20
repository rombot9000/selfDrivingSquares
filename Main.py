#!/opt/local/bin/python3.5

from selfDriving.Shape import dataType
from selfDriving.Rectangle import Rectangle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation, FFMpegWriter

def update(i):
    reactionTime = 2
    for rectangle in listOfRectangles:
        rectangle.run(reactionTime)
    for j in range(0, numberOfRectangles):
        rectangle = listOfRectangles[j]
        edges = rectangle.trajectory[dataType.edges][-1]
        plgn[j].set_xy([edges[0][0], edges[0][1], edges[1][1], edges[1][0]])
        plts[j][0].set_data(*zip(*rectangle.trajectory[dataType.center]))
        trgt[j].set_offsets([rectangle.target[0], rectangle.target[1]])

def setup():
    for rectangle in listOfRectangles:
        edges = rectangle.trajectory[dataType.edges][0]
        plgn.append(ax.add_patch(Polygon([edges[0][0], edges[0][1], edges[1][1], edges[1][0]], closed=True, fill=True, color = rectangle.color)))
        plts.append(ax.plot(*zip(*rectangle.trajectory[dataType.center]), color=rectangle.color, linestyle='dotted'))
        trgt.append(ax.scatter(rectangle.target[0], rectangle.target[1], color=rectangle.color, marker='x'))  

if __name__ == "__main__":
    # ---------------------------
    # Calculate single trajectory
    # ---------------------------
    listOfRectangles = []
    numberOfRectangles = 4
    for i in range(0,numberOfRectangles):
        listOfRectangles.append(Rectangle())
    # ----------------------------
    # Show animation of trajectory
    # ----------------------------
    fig = plt.figure()
    ax  = fig.add_subplot(111)
    ax.set_aspect(1)
    ax.axis([-100,100,-100,100])
    plgn = []
    plts = []
    trgt = [] 
    setup()
    anim = FuncAnimation(fig, update, None, interval=100)
    plt.show()
    
    # Raises error -> use show() for now
    #mywriter = FFMpegWriter(fps=60)
    #anim.save('trajectory.mp4',writer=mywriter)