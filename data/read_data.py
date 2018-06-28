from __future__ import print_function, division
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import LogisticRegression
from sklearn.neural_network import MLPRegressor
import numpy as np



# 1.29310650083 470 [3.5822370462661737, 0.6119967377262815] False
# [2.7392222025446036, 0.37957370143594843, 0.8121965278913195]


# Convert data into Numpy arrays
alternate =True
inputs = []
outputs = []

with open("new_output.txt", 'r') as f:
    for line in f:
        if alternate:
            params = line.split(" ")
            params[0] = float(params[0])
            params[1] = int(params[1])
            params[2] = float(params[2][1:-1])
            params[3] = float(params[3][:-1])
            params[4] = params[4][:-1]
            if params[4] == "True":
                params[4] = True
            else:
                params[4] = False
        else:
            out = line.split(" ")
            out[0] = float(out[0][1:-1])
            out[1] = float(out[1][:-1])
            out[2] = float(out[2][:-2])
            inputs.append(params)
            outputs.append(out)
        alternate = not alternate

X = np.array(inputs)
y = np.array(outputs)





# Multi-layer Perceptron regressor

mlp = MLPRegressor(hidden_layer_sizes=(100,),  activation='relu', solver='adam',    alpha=0.001,batch_size='auto',
               learning_rate='constant', learning_rate_init=0.01, power_t=0.5, max_iter=1000, shuffle=True,
               random_state=None, tol=0.0001, verbose=False, warm_start=False, momentum=0.9,
               nesterovs_momentum=True, early_stopping=False, validation_fraction=0.1, beta_1=0.9, beta_2=0.999,
               epsilon=1e-08)
mlp.fit(X, y)

mlp_dist_error = 0
mlp_offset_error = 0
mlp_orn_error = 0

data_points = X.shape[0]

for i in range(data_points):
    # print("--------------------------------------------------------------------------")
    # print("Data point: ", X[i:i+1,:])
    # print("Prediction: ", mlp.predict(X[i:i+1,:]))
    # print("Actual:     ", y[i:i+1,:])
    mlp_dist_error += abs(mlp.predict(X[i:i+1,:])[0][0] - y[i:i+1,:][0][0])
    mlp_offset_error += abs(mlp.predict(X[i:i+1,:])[0][1] - y[i:i+1,:][0][1])
    mlp_orn_error += abs(mlp.predict(X[i:i+1,:])[0][2] - y[i:i+1,:][0][2])

avg_mlp_dist_error = mlp_dist_error/data_points
avg_mlp_offset_error = mlp_offset_error/data_points
avg_mlp_orn_error = mlp_orn_error/data_points
print("******************************************************************")
print("MLP REGRESSOR")
print("******************************************************************")
print("Avg. Distance Error: ", avg_mlp_dist_error)
print("Avg. Offset Error: ", avg_mlp_offset_error)
print("Avg. Orientation Error: ", avg_mlp_orn_error)
print("------------------------------------------------------------------")
print("Score: ", mlp.score(X, y, sample_weight=None))







# Linear regressor

lr = LinearRegression(fit_intercept=True, normalize=False, copy_X=True, n_jobs=1)

lr.fit(X, y, sample_weight=None)

lr_dist_error = 0
lr_offset_error = 0
lr_orn_error = 0

data_points = X.shape[0]

for i in range(data_points):
    # print("--------------------------------------------------------------------------")
    # print("Data point: ", X[i:i+1,:])
    # print("Prediction: ", lr.predict(X[i:i+1,:]))
    # print("Actual:     ", y[i:i+1,:])
    lr_dist_error += abs(lr.predict(X[i:i+1,:])[0][0] - y[i:i+1,:][0][0])
    lr_offset_error += abs(lr.predict(X[i:i+1,:])[0][1] - y[i:i+1,:][0][1])
    lr_orn_error += abs(lr.predict(X[i:i+1,:])[0][2] - y[i:i+1,:][0][2])

avg_lr_dist_error = lr_dist_error/data_points
avg_lr_offset_error = lr_offset_error/data_points
avg_lr_orn_error = lr_orn_error/data_points
print("******************************************************************")
print("LINEAR REGRESSOR")
print("******************************************************************")
print("Avg. Distance Error: ", avg_lr_dist_error)
print("Avg. Offset Error: ", avg_lr_offset_error)
print("Avg. Orientation Error: ", avg_lr_orn_error)
print("------------------------------------------------------------------")
print("Score: ", lr.score(X, y, sample_weight=None))

