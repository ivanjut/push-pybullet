import numpy as np
# 4.56329396278 323 [1.2109551086938002, 0.603144269927146]
# (4.554353379504424, -0.44960224385343023)

alternate =True
inputs = []
outputs = []

with open("test_output.txt", 'r') as f:
    for line in f:
        if alternate:
            temp = line.split(" ")
            print "first" + str(temp)
            temp[1] = temp[1][1:-1]
            temp[2] = temp[2][0:-2]
        else:
            temp2 = line.split(" ")
            print "second" + str(temp2)
            temp2[0] = temp2[0][1:-1]
            temp2[1] = temp2[1][:-2]
            inputs.append([int(temp[0]),float(temp[1]),float(temp[2])])
            outputs.append([float(temp2[0]),float(temp2[1])])
        alternate = not alternate

X = np.array(inputs)
y = np.array(outputs)


from sklearn.linear_model import LinearRegression
from sklearn.linear_model import LogisticRegression
from sklearn.neural_network import MLPRegressor


clf = MLPRegressor(hidden_layer_sizes=(10,),  activation='relu', solver='adam',    alpha=0.001,batch_size='auto',
               learning_rate='constant', learning_rate_init=0.01, power_t=0.5, max_iter=1000, shuffle=True,
               random_state=None, tol=0.0001, verbose=False, warm_start=False, momentum=0.9,
               nesterovs_momentum=True, early_stopping=False, validation_fraction=0.1, beta_1=0.9, beta_2=0.999,
               epsilon=1e-08)
clf.fit(X, y)

for i in range(100):
    print "-"
    print X[i:i+1,:]
    print clf.predict(X[i:i+1,:])
    print y[i:i+1,:]