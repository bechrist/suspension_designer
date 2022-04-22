import networkx as nx
import numpy as np

class KinematicFrame(nx.DiGraph):
    def __init__(self, Name     = "World", \
                       Key      = "O", \
                       Base     = "", \
                       Follower = "", \
                       Origin   = np.zeros((3,1)), \
                       Rotation = np.zeros((3,1)), \
                       PoI      = {"O" : np.zeros((3,1))    , \
                                   "E1": np.array([1, 0, 0]), \
                                   "E2": np.array([0, 1, 0]), \
                                   "E3": np.array([0, 0, 1])} ):



        self[Key]._Name     = Name

        self[Key]._Base     = Base
        self[Key]._Follower = Follower

        self[Key].Origin   = Origin
        self[Key].Rotation = Rotation

        self[Key].PoI      = PoI