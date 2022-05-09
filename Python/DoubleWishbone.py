import numpy             as np
import matplotlib.pyplot as plt
import networkx          as nx

from   copy     import deepcopy

# %% Dictionary Constructors
def PoI(Title, Position, Style):
    return {'Title'   : Title, 
            'Position': Position, 
            'Style'   : Style}

def Frame(Title, Base, Position, Rotation, DoF, PoI):
    return {'Title'   : Title,
            'Base'    : Base, 
            'Position': Position,
            'Rotation': Rotation, 
            'DoF'     : np.array(DoF, dtype=bool),
            'PoI'     : PoI}

# %% System Initialization 
def DoubleWishboneInit(self):
    # Generates default multibody frame graph and sample structure of double 
    # wishbone linkage.

    ## Generate Multibody System Graph
    # Default Point of Interest Dictionary
    DefPoI = dict()

    DefPoI['O']  = PoI('Origin', np.zeros(3)       , 'k.')
    DefPoI['E1'] = PoI('x-Axis', np.array([25,0,0]), 'k.')
    DefPoI['E2'] = PoI('y-Axis', np.array([0,25,0]), 'k.')
    DefPoI['E3'] = PoI('z-Axis', np.array([0,0,25]), 'k.')

    # Intermediate Frame
    I = Frame('Intermediate', '', np.zeros(3), np.zeros(3), 
        [0,0,0,0,0,0], deepcopy(DefPoI))

    I['PoI']['RC'] = PoI('Roll Center'         , np.zeros(3), 'kx')
    I['PoI']['FC'] = PoI('Front Instant Center', np.zeros(3), 'k*')

    I['PoI']['PC'] = PoI('Pitch Center'        , np.zeros(3), 'kx')
    I['PoI']['SC'] = PoI('Side Instant Center' , np.zeros(3), 'k*')

    # Tire Frame
    T = Frame('Tire', 'I', np.zeros(3), np.zeros(3), 
        [1,1,0,1,0,1], deepcopy(DefPoI))

    # Wheel Frame
    W = Frame('Wheel', 'T', np.zeros(3), np.zeros(3), 
        [0,0,0,0,1,0], deepcopy(DefPoI))

    W['PoI']['LB'] = PoI('Lower Pickup'  , np.zeros(3), 'ks')
    W['PoI']['UB'] = PoI('Upper Pickup'  , np.zeros(3), 'ks')
    W['PoI']['TB'] = PoI('Tie Rod Pickup', np.zeros(3), 'ks')

    # Body Frame
    B = Frame('Body', 'I', np.zeros(3), np.zeros(3), 
        [0,0,1,1,1,0], deepcopy(DefPoI))

    # Axle Frame
    X = Frame('Axle', 'B', np.zeros(3), np.zeros(3), 
        [0,0,0,0,0,0], deepcopy(DefPoI))

    X['PoI']['LAF'] = PoI('Lower A-Arm Front Pickup', np.zeros(3), 'ks')
    X['PoI']['LAR'] = PoI('Lower A-Arm Rear Pickup' , np.zeros(3), 'ks')
    X['PoI']['UAF'] = PoI('Upper A-Arm Front Pickup', np.zeros(3), 'ks')
    X['PoI']['UAR'] = PoI('Upper A-Arm Rear Pickup' , np.zeros(3), 'ks')
    X['PoI']['TA']  = PoI('Tie Rod Pickup'          , np.zeros(3), 'ks')

    # Suspension Linkage Frames
    LA = Frame('Lower A-Arm', 'X', np.zeros(3), np.zeros(3), 
        [0,0,0,1,0,0], deepcopy(DefPoI))

    LA['PoI']['LAF'] = PoI('Front Pickup', np.zeros(3), 'ko')
    LA['PoI']['LAR'] = PoI('Rear Pickup' , np.zeros(3), 'ko')
    LA['PoI']['LB']  = PoI('Apex'        , np.zeros(3), 'ko')

    UA = Frame('Upper A-Arm', 'X', np.zeros(3), np.zeros(3), 
        [0,0,0,1,0,0], deepcopy(DefPoI))

    UA['PoI']['UAF'] = PoI('Front Pickup', np.zeros(3), 'ko')
    UA['PoI']['UAR'] = PoI('Rear Pickup' , np.zeros(3), 'ko')
    UA['PoI']['UB']  = PoI('Apex'        , np.zeros(3), 'ko')

    TR = Frame('Tie Rod', 'X', np.zeros(3), np.zeros(3),
        [0,0,0,1,0,0], deepcopy(DefPoI))
    
    TR['PoI']['TB'] = PoI('Outer Pickup', np.zeros(3), 'ko')

    # Populate Graph Nodes
    self.add_nodes_from([
        ('I' , I ),
        ('T' , T ), 
        ('W' , W ),
        ('B' , B ),
        ('X' , X ),
        ('LA', LA),
        ('UA', UA),
        ('TR', TR),
    ])

    # Populate Graph Edges
    self.add_edges_from([
        ('I','T'), 
        ('T','W'), 
        ('I','B'), 
        ('B','X'), 
        ('X','LA'), 
        ('X','UA'), 
        ('X','TR')
    ])
    
    # Generate Shortest Path Tree
    self.Path = nx.shortest_path(self)

    ## Generate Sample Structure
    # This is extremely dependent on the choice of design
    # rules applied during the design generation process. Please
    # refer to DoubleWishboneDesign() for more details.
    #
    # Key:
    # 1: Value is Sampled
    # 0: Value is Auto-Calculated or Fixed
    
    # Default Sample Structure
    self.Sample['Linkage'] = dict()

    self.Sample['Linkage']['LAF'] = np.array([1,1,0])
    self.Sample['Linkage']['LAR'] = np.array([1,1,0])
    self.Sample['Linkage']['UAF'] = np.array([1,0,0])
    self.Sample['Linkage']['UAR'] = np.array([1,0,0])
    self.Sample['Linkage']['TA']  = np.array([1,1,0])

    self.Sample['Linkage']['LB']  = np.array([1,1,1])
    self.Sample['Linkage']['UB']  = np.array([1,0,1])
    self.Sample['Linkage']['TB']  = np.array([1,1,1])

    # Parsing Bound Restricted Values
    for p in list(self.Sample['Linkage'].keys()):
        for i in np.argwhere(self.Sample['Linkage'][p]==1):
            if (np.diff(self.Bound['Linkage'][p][i])==0)[0]:
                self.Sample['Linkage'][p][i] = 0

    return self

# %% System Generation


# %% System Solver