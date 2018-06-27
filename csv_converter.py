from __future__ import print_function
import csv

alternate = True
data = [["iters", "x_rot", "y_rot", "dist", "offset"]]

with open("test_output.txt", 'r') as f:
    for line in f:
        if alternate:
            param = line.split(" ")
            param[1] = param[1][1:-1]
            param[2] = param[2][0:-2]
            # print("Params: " + str(param))
        else:
            result = line.split(" ")
            result[0] = result[0][1:-1]
            result[1] = result[1][:-2]
            # print("Result: " + str(result))
            data_point = [int(param[0]), float(param[1]), float(param[2]), float(result[0]), float(result[1])]
            # print("Data point: ", data_point)
            data.append(data_point)
        alternate = not alternate

with open('output.csv', 'w') as data_file:
    writer = csv.writer(data_file)
    writer.writerows(data)


