# -*- coding: utf-8 -*-

import numpy as np
import os

from . import SVR

class BumperModel:
    def __init__(self, folder=None):
        if folder is None:
            folder = os.path.dirname(__file__)

        self.models = [
            SVR(os.path.join(folder, 'trainedModels_Fx.npz')),
            SVR(os.path.join(folder, 'trainedModels_Fy.npz')),
            SVR(os.path.join(folder, 'trainedModels_Tz.npz'))
        ]
    
    def predict(self, x):
        return [model.predict(x) for model in self.models]
