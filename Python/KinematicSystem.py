# %% KinematicSystem 
import numpy             as np
import networkx          as nx
import datetime

from Resources import GeometrySolvers as geo
from Linkages  import DoubleWishbone 

class KinematicSystem(nx.Graph):
    # Main class within suspension designer responsible for keeping track of 
    # kinematic frames and associated points of interest within a graph
    # structure.

    def __init__(self, Name=str(), Target=dict(), Bound=dict()):
        # Run Super Class Initialization
        super(KinematicSystem, self).__init__()

        # Set Properties
        self.Name   = Name
        self.Date   = datetime.datetime.now()
        
        self.Target = Target
        self.Bound  = Bound
        self.Sample = dict()

        # Linkage Initialization
        LinkageInit = {"Double Wishbone": DoubleWishbone.Init}

        self = LinkageInit[self.Target['Type']['Linkage']](self)  

        # Strut Initialization   
        StrutInit = {}

        # Spring Initialization    
        SpringInit = {}

    def GenerateDesign(self, System='All'):
        if System == 'All':
            System = list(self.Bound.keys())
        elif isinstance(System, str):
            System = System.split()

        # Generation Function Pointer Dictionaries
        LinkageDesign = {"Double Wishbone": DoubleWishbone.Design}
        
        StrutDesign = {}

        SpringDesign = {}

        for Sys in System:
            if Sys == 'Linkage':
                self = LinkageDesign[self.Target['Type']['Linkage']](self)
            elif Sys == 'Strut':
                self = StrutDesign[self.Target['Type']['Strut']](self)
            elif Sys == 'Spring':
                self = SpringDesign[self.Target['Type']['Spring']](self) 