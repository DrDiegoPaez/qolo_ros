from prediction_model import *
import numpy as np
import os

folder = os.path.dirname(__file__)
# svr = SVR(os.path.join(folder, 'prediction_model', 'trainedModels_Fx.npz'))

# x = np.asarray([
#     [4.4650, -13.5988, -0.0045, -0.0386, -0.0169],
#     [4.2598, -12.9745, -0.0037, -0.0366, -0.0160],
#     [4.1270, -12.5698, -0.0028, -0.0352, -0.0154],
#     [4.0766, -12.4143, -0.0018, -0.0345, -0.0150],
#     [4.1059, -12.4998, -0.0008, -0.0344, -0.0148],
#     [9.2007, -52.7847,  1.0001,  2.0350,  1.0149],
# ])
# print(svr.predict_all(x))

x_single = np.asarray([
    [4.4650, -0.5988, -0.0045, 0.2365, -0.0386, -0.0169],
])
# print(svr.predict(x_single))

from timeit import default_timer as timer

# start = timer()
# n = 1000
# for i in range(n):
#     y = svr.predict(x_single)
# end = timer()
# ellapsed = end - start
# print("Total Elapsed Time for {:d} calls = {:.5f} s".format(n, ellapsed))
# print("Avg. Elapsed Time per call = {:.5f} s".format(ellapsed/n))

# Prediction Models
models = BumperModel()
print(models.predict(x_single))

start = timer()
n = 1000
for i in range(n):
    y = models.predict(x_single)
end = timer()
print(y)
ellapsed = end - start
print("Total Elapsed Time for {:d} calls = {:.5f} s".format(n, ellapsed))
print("Avg. Elapsed Time per call = {:.5f} s".format(ellapsed/n))