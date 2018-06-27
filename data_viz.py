from __future__ import print_function
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.axes_grid1 import make_axes_locatable
import seaborn as sns
import math

PI = math.pi

data = pd.read_csv("output.csv")

fig, axes = plt.subplots(nrows=2, ncols=1)

axes[0].scatter(x=data["iters"], y=data["dist"], s=0.1, c='b')
axes[0].set_xlim((200, 600))
axes[0].set_ylim((4, 7))
axes[0].set_xlabel("Iterations")
axes[0].set_ylabel("Distance")
axes[0].set_title("Iterations vs. Straight Line Distance")

axes[1].scatter(x=data["x_rot"], y=data["y_rot"], s=5, c=data["dist"], alpha=0.4)
axes[1].set_xlim((0, 2*PI))
axes[1].set_ylim((0, PI/2))
axes[1].set_xlabel("X Rotation")
axes[1].set_ylabel("Y Rotation")
axes[1].set_title("Gripper Orientation vs. Straight Line Distance")

plt.tight_layout()
plt.show()