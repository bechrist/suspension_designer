# %% KinematicSystem 
import networkx as nx
import numpy    as np
import datetime

from DoubleWishbone import *

class KinematicSystem(nx.Graph):
    # Main class within suspension designer responsible for keeping track of 
    # kinematic frames and associated points of interest within a graph
    # structure.

    def __init__(self, Name=str(), Target=dict(), Bound=dict()):
        # Set Properties
        self.Name   = Name
        self.Date   = datetime.datetime.now()
        self.Target = Target
        self.Bound  = Bound

        # Linkage Initialization Method
        LinkageInit = {"Double Wishbone": DoubleWishboneInit}

        self = LinkageInit[self.Target['Type']['Linkage']](self)     