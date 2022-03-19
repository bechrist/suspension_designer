import numpy as np

class KinematicFrame():
    def __init__(self, Name     = "World", \
                       Tag      = "O", \
                       Base     = "", \
                       Origin   = np.zeros((3,1)), \
                       Rotation = np.zeros((3,1)), \
                       Follower = "", \
                       PoI      = {'O': np.zeros((3,1)), \
                                   "X": np.array([1, 0, 0]), \
                                   "Y": np.array([0, 1, 0]), \
                                   "Z": np.array([0, 0, 0])} ):
        self[Tag].Name     = Name

        self[Tag].Base     = Base

        self[Tag].Origin   = Origin
        self[Tag].Rotation = Rotation

        self[Tag].Follower = Follower

        self[Tag].PoI      = PoI