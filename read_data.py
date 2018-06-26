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
            temp[2] = temp[2][1:-1]
            temp[3] = temp[3][0:-2]
        else:
            temp2 = line.split(" ")
            print "second" + str(temp2)
            temp2[0] = temp2[0][1:-1]
            temp2[1] = temp2[1][:-2]
            inputs.append([float(temp[0]),int(temp[1]),float(temp[2]),float(temp[3])])
            outputs.append([float(temp2[0]),float(temp2[1])])
        alternate = not alternate

x = np.array(inputs)
y = np.array(outputs)