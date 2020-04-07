from decision_making.Coordinator import Coordinator
from decision_making.trajectory.Path import Path
from representations.StablePose import StablePose


path_1 = Path([StablePose(0.0, 0.0, 0.0), StablePose(0.0, 0.0, 0.5), StablePose(0.0, 0.0, 1.0)])
path_2 = Path([StablePose(0.16, -0.16, 0.5), StablePose(0.16, 0.16, 0.5),
               StablePose(-0.16, 0.16, 0.5)])
paths = {1: path_1, 2: path_2}

c = Coordinator()
paths = c.coordinate(paths)

x_i = [v[0] for v in c.get_x()]
x_j = [v[1] for v in c.get_x()]
c.get_delta().plot(1, 2, (x_i, x_j))
print(paths[1])
print(paths[2])
