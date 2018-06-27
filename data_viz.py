from __future__ import print_function
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import seaborn as sns

data = pd.read_csv("output.csv")

print(data.head())
