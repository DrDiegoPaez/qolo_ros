import numpy as np
import yaml
import os

class SVR:
    def __init__(self, filename):
        if ".yaml" in filename:
            with open(filename, 'r') as f:
                model = yaml.load(f, Loader=yaml.FullLoader)
        elif ".npz" in filename:
            model = np.load(filename)
        self.mu = np.array(model['mu'])
        self.sigma = np.array(model['sigma'])
        self.sv = np.array(model['sv'])
        self.kernel_scale = np.array(model['kernel_scale'])
        self.alpha = np.array(model['alpha'])
        self.bias = np.array(model['bias'])

    def predict_all(self, x):
        n = x.shape[0]
        y = np.zeros((n, 1))
        for i in range(n):
            y[i] = self.predict(x[i:i+1,:])
        return y

    def predict(self, x):
        if x.shape[0] > 1:
            raise ValueError(
                "Input shape of {:d} is not supported by SVR.predict()".format(x.shape[0]) +
                "\n\tUse SVR.predict_all() for multiple input samples"
            )

        if x.shape[1] != 6:
            raise ValueError(
                "Input shape of {:d} is different from expected dimension of {:d}".format(
                    x.shape[1], self.in_dim
                )
            )
        x = np.delete(x, 2, 1)
        # x[0, 2] = 10

        x_normalised = (x - self.mu) / self.sigma

        _temp = (self.sv - x_normalised) / self.kernel_scale
        K = np.exp(- np.linalg.norm(_temp, ord=2, axis=1)**2)

        y = np.sum(self.alpha * K) + self.bias

        return y


class BumperModel:
    def __init__(self, folder=None):
        if folder is None:
            folder = os.path.dirname(__file__)

        # self.models = [
        #     SVR(os.path.join(folder, 'trainedModels_Fx.yaml')),
        #     SVR(os.path.join(folder, 'trainedModels_Fy.yaml')),
        #     SVR(os.path.join(folder, 'trainedModels_Tz.yaml'))
        # ]
        self.models = [
            SVR(os.path.join(folder, 'trainedModels_Fx.npz')),
            SVR(os.path.join(folder, 'trainedModels_Fy.npz')),
            SVR(os.path.join(folder, 'trainedModels_Tz.npz'))
        ]
    
    def predict(self, x):
        return [model.predict(x) for model in self.models]


if __name__ == "__main__":
    svr = SVR('trainedModels_Fx.npz')

    x = np.matrix([
        [4.4650, -13.5988, 0.0108, -0.0045, -0.0386, -0.0169],
        [4.2598, -12.9745, 0.0361, -0.0037, -0.0366, -0.0160],
        [4.1270, -12.5698, 0.0613, -0.0028, -0.0352, -0.0154],
        [4.0766, -12.4143, 0.0832, -0.0018, -0.0345, -0.0150],
        [4.1059, -12.4998, 0.0989, -0.0008, -0.0344, -0.0148],
        [4.2007, -12.7847, 0.1060,  0.0001, -0.0350, -0.0149],
    ])
    print(svr.predict_all(x))
    
    x_single = np.matrix([
        [4.4650, -13.5988, 0.0108, -0.0045, -0.0386, -0.0169],
    ])
    print(svr.predict(x_single))

    from timeit import default_timer as timer

    start = timer()
    n = 1000
    for i in range(n):
        y = svr.predict(x_single)
    end = timer()
    ellapsed = end - start
    print("Total Elapsed Time for {:d} calls = {:.5f} s".format(n, ellapsed))
    print("Avg. Elapsed Time per call = {:.5f} s".format(ellapsed/n))
    
    # Prediction Models
    models = BumperModel()
    print(models.predict(x_single))

    start = timer()
    n = 1000
    for i in range(n):
        y = models.predict(x_single)
    end = timer()
    ellapsed = end - start
    print("Total Elapsed Time for {:d} calls = {:.5f} s".format(n, ellapsed))
    print("Avg. Elapsed Time per call = {:.5f} s".format(ellapsed/n))
