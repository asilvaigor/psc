from decision_making.Delta import Delta
from decision_making.trajectory.Path import Path
from representations.StablePose import StablePose


path_1 = Path([StablePose(0.0, 0.0, 0.0), StablePose(0.0, 0.0, 0.5), StablePose(0.0, 0.0, 1.0)])
path_2 = Path([StablePose(0.16, -0.16, 0.5), StablePose(0.16, 0.16, 0.5),
               StablePose(-0.16, 0.16, 0.5)])
paths = {1: path_1, 2: path_2}

delta = Delta(paths)
delta.plot(1, 2)
