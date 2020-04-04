from decision_making.Delta import Delta
from decision_making.trajectory.Path import Path
from representations.StablePose import StablePose


path_1 = Path([StablePose(-0.00058308581356, -0.000597334175836, 0.0314681529999),
               StablePose(-0.0250881500089, -0.238212624657, 0.265956244637),
               StablePose(-0.0324567332286, -0.394365086039, 0.628581479016),
               StablePose(0.360684212199, -0.753693566673, 0.75051833137),
               StablePose(0.2, -0.8, 1.0)])
path_2 = Path([StablePose(0.49904987216, -0.000903245061636, 0.0314985252917),
               StablePose(0.49700777221, 0.0752679586831, 0.197484733984),
               StablePose(0.155659067958, -0.0950697263561, 0.45275366819),
               StablePose(-0.0324567332286, -0.394365086039, 0.628581479016),
               StablePose(-0.130773142371, -0.530582970581, 0.991440620386),
               StablePose(-0.2, -0.8, 1.0)])
path_3 = Path([StablePose(0.0, 0.0, 0.0), StablePose(0.0, 0.0, 0.5), StablePose(0.0, 0.0, 1.0)])
path_4 = Path([StablePose(0.0, -1, 0.5), StablePose(0.0, 1, 0.5)])
paths = {1: path_1, 2: path_2}

delta = Delta(paths)
print(path_1.intersections)
print(path_2.intersections)
