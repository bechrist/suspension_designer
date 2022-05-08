# %% KinematicSystem 
import networkx as nx
import numpy as np
import datetime

import DoubleWishbone

class KinematicSystem(nx.Graph):
    def __init__(self, Name=str(), Target=dict(), Bound=dict()):
        # Set Properties
        self.Name   = Name
        self.Date   = datetime.datetime.now()
        self.Target = Target
        self.Bound  = Bound

        # Linkage Initialization Method
        LinkageInit = {"Double Wishbone": DoubleWishboneInit}
        
        

        