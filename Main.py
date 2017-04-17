#!/opt/local/bin/python3.5

from selfDriving.Rectangle import Rectangle, dataType
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation, FFMpegWriter

def update(i):
    for j in range(0, numberOfRectangles):
        edges = listOfRectangles[j].trajectory[dataType.edges][i]
        plgn[j].set_xy([edges[0][0], edges[0][1], edges[1][1], edges[1][0]])

if __name__ == "__main__":
    # ---------------------------
    # Calculate single trajectory
    # ---------------------------
    listOfRectangles = []
    numberOfRectangles = 10
    for i in range(0,numberOfRectangles):
        listOfRectangles.append(Rectangle())
    reactionTime = 4
    while listOfRectangles[0].targetCounter < 4:
        for rectangle in listOfRectangles:
            rectangle.run(reactionTime)
    # ----------------------------
    # Show animation of trajectory
    # ----------------------------
    fig = plt.figure()
    ax  = fig.add_subplot(111)
    ax.set_aspect(1)
    ax.axis([-100,100,-100,100])
    plgn = []
    for rectangle in listOfRectangles:
        edges = rectangle.trajectory[dataType.edges][0]
        plgn.append(ax.add_patch(Polygon([edges[0][0], edges[0][1], edges[1][1], edges[1][0]], closed=True, fill=False)))
        ax.plot(*zip(*rectangle.trajectory[dataType.center]))        
    anim = FuncAnimation(fig, update, frames=np.arange(0, len(listOfRectangles[0].trajectory[dataType.center])), interval=100)
    plt.show()
    
    # Raises error -> use show() for now
    #mywriter = FFMpegWriter(fps=60)
    #anim.save('trajectory.mp4',writer=mywriter)