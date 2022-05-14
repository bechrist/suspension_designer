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
def Init(self):
    # Generates default multibody frame graph and sample structure of double 
    # wishbone linkage.

    ## Generate Multibody System Graph
    # Default Point of Interest Dictionary
    DefPoI = dict()

    DefPoI['O']  = PoI('Origin', np.zeros((3,1))       , 'k.')
    DefPoI['E1'] = PoI('x-Axis', np.array([[25,0,0]]).T, 'k.')
    DefPoI['E2'] = PoI('y-Axis', np.array([[0,25,0]]).T, 'k.')
    DefPoI['E3'] = PoI('z-Axis', np.array([[0,0,25]]).T, 'k.')

    # Intermediate Frame
    I = Frame('Intermediate', '', np.zeros((3,1)), np.zeros((3,1)), 
              [0,0,0,0,0,0], deepcopy(DefPoI))

    I['PoI']['RC'] = PoI('Roll Center'         , np.zeros((3,1)), 'kx')
    I['PoI']['FC'] = PoI('Front Instant Center', np.zeros((3,1)), 'k*')

    I['PoI']['PC'] = PoI('Pitch Center'        , np.zeros((3,1)), 'kx')
    I['PoI']['SC'] = PoI('Side Instant Center' , np.zeros((3,1)), 'k*')

    # Tire Frame
    T = Frame('Tire', 'I', np.zeros((3,1)), np.zeros((3,1)), 
              [1,1,0,1,0,1], deepcopy(DefPoI))

    # Wheel Frame
    W = Frame('Wheel', 'T', np.zeros((3,1)), np.zeros((3,1)), 
              [0,0,0,0,1,0], deepcopy(DefPoI))

    W['PoI']['LB'] = PoI('Lower Pickup'  , np.zeros((3,1)), 'ks')
    W['PoI']['UB'] = PoI('Upper Pickup'  , np.zeros((3,1)), 'ks')
    W['PoI']['TB'] = PoI('Tie Rod Pickup', np.zeros((3,1)), 'ks')

    # Body Frame
    B = Frame('Body', 'I', np.zeros((3,1)), np.zeros((3,1)), 
              [0,0,1,1,1,0], deepcopy(DefPoI))

    # Axle Frame
    X = Frame('Axle', 'B', np.zeros((3,1)), np.zeros((3,1)), 
              [0,0,0,0,0,0], deepcopy(DefPoI))

    X['PoI']['LAF'] = PoI('Lower A-Arm Front Pickup', np.zeros((3,1)), 'ks')
    X['PoI']['LAR'] = PoI('Lower A-Arm Rear Pickup' , np.zeros((3,1)), 'ks')
    X['PoI']['UAF'] = PoI('Upper A-Arm Front Pickup', np.zeros((3,1)), 'ks')
    X['PoI']['UAR'] = PoI('Upper A-Arm Rear Pickup' , np.zeros((3,1)), 'ks')
    X['PoI']['TA']  = PoI('Tie Rod Pickup'          , np.zeros((3,1)), 'ks')

    # Suspension Linkage Frames
    LA = Frame('Lower A-Arm', 'X', np.zeros((3,1)), np.zeros((3,1)), 
               [0,0,0,1,0,0], deepcopy(DefPoI))

    LA['PoI']['LAF'] = PoI('Front Pickup', np.zeros((3,1)), 'ko')
    LA['PoI']['LAR'] = PoI('Rear Pickup' , np.zeros((3,1)), 'ko')
    LA['PoI']['LB']  = PoI('Apex'        , np.zeros((3,1)), 'ko')

    UA = Frame('Upper A-Arm', 'X', np.zeros((3,1)), np.zeros((3,1)), 
               [0,0,0,1,0,0], deepcopy(DefPoI))

    UA['PoI']['UAF'] = PoI('Front Pickup', np.zeros((3,1)), 'ko')
    UA['PoI']['UAR'] = PoI('Rear Pickup' , np.zeros((3,1)), 'ko')
    UA['PoI']['UB']  = PoI('Apex'        , np.zeros((3,1)), 'ko')

    TR = Frame('Tie Rod', 'X', np.zeros((3,1)), np.zeros((3,1)),
               [0,0,0,1,0,0], deepcopy(DefPoI))
    
    TR['PoI']['TB'] = PoI('Outer Pickup', np.zeros((3,1)), 'ko')

    # Populate Graph Nodes
    self.add_nodes_from([
        ('I' , I ),
        ('T' , T ), 
        ('W' , W ),
        ('B' , B ),
        ('X' , X ),
        ('LA', LA),
        ('UA', UA),
        ('TR', TR)
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

    self.Sample['Linkage']['LAF'] = np.array([[1,1,0]]).T
    self.Sample['Linkage']['LAR'] = np.array([[1,1,0]]).T
    self.Sample['Linkage']['UAF'] = np.array([[1,0,0]]).T
    self.Sample['Linkage']['UAR'] = np.array([[1,0,0]]).T
    self.Sample['Linkage']['TA']  = np.array([[1,1,0]]).T

    self.Sample['Linkage']['LB']  = np.array([[1,1,1]]).T
    self.Sample['Linkage']['UB']  = np.array([[1,0,1]]).T
    self.Sample['Linkage']['TB']  = np.array([[1,1,1]]).T

    # Parsing Bound Restricted Values
    for P in list(self.Sample['Linkage'].keys()):
        r = (np.diff(self.Bound['Linkage'][P]) == 0)
        self.Sample['Linkage'][P][r] = 0

    return self

# %% System Generation
def Design(self):
    # Populates a multibody frame graph with the static frame
    # transforms and point of interest locations based on the design
    # targets and sample of the design space described by the bounds.

    ## Allocate Known Parameters   
    # Tire Frame
    if self.Target['Type']['Axle'] == 'Front':
        self.nodes['T']['Position'][0] = self.Target['Vehicle']['Wheelbase'] \
            * (1 - self.Target['Vehicle']['WeightDist'][0]/100)
    elif self.Target['Type']['Axle'] == 'Rear':
        self.nodes['T']['Position'][0] = self.Target['Vehicle']['Wheelbase'] \
            * -self.Target['Vehicle']['WeightDist'][0]/100   
    else:
        raise Exception(('Axle type not recognized. Please input \'Front\''
                         'or \'Rear\'.'))

    self.nodes['T']['Position'][1] = self.Target['Linkage']['Track']/2

    self.nodes['T']['Rotation'][0] = -self.Target['Linkage']['Camber']
    self.nodes['T']['Rotation'][2] =  self.Target['Linkage']['Toe']
    
    # Wheel Frame
    self.nodes['W']['Position'][2] = self.Target['Vehicle']['LoadedRadius'] \
        / np.sin(np.pi/2 - self.Target['Linkage']['Camber'])

    self.nodes['W']['Rotation'][1] = -self.Target['Linkage']['Caster']

    # Body Frame
    self.nodes['B']['Position'][2] = self.Target['Vehicle']['CG'][2]
    self.nodes['B']['Rotation'][1] = self.Target['Vehicle']['Rake']

    # Axle Frame
    self.nodes['X']['Position'][0] = self.nodes['T']['Position'][0]
    self.nodes['X']['Position'][2] = self.Target['Vehicle']['Ride'] \
        - self.Target['Vehicle']['CG'][2]

    ## Design Space Sampling
    for P in ['LAF','LAR','UAF','UAR','TA','LB','UB','TB']:
        if P[1] == 'A':
            F = 'X'
        elif P[1] == 'B':
            F = 'W'
        
        self.nodes[F]['PoI'][P]['Position'] = self.Bound['Linkage'][P][:,[0]] \
            + self.Sample['Linkage'][P] * np.diff(self.Bound['Linkage'][P])

        if P == 'LAR':
            if all(self.Bound['Linkage']['LAR'][1] == 0):
                self.nodes['X']['PoI']['LAR']['Position'][1] = \
                   self.nodes['X']['PoI']['LAF']['Position'][1]

                self.Bound['Linkage']['LAR'][1,:] = \
                    self.Bound['Linkage']['LAF'][1,:]   
        elif P == 'UAF':
            if all(self.Bound['Linkage']['UAF'][0] == 0):
                self.nodes['X']['PoI']['UAF']['Position'][0] = \
                   self.nodes['X']['PoI']['LAF']['Position'][0]

                self.Bound['Linkage']['UAF'][0,:] = \
                    self.Bound['Linkage']['LAF'][0,:]   
        elif P == 'UAR':
            if all(self.Bound['Linkage']['UAR'][0] == 0):
                self.nodes['X']['PoI']['UAR']['Position'][0] = \
                   self.nodes['X']['PoI']['LAR']['Position'][0]

                self.Bound['Linkage']['UAR'][0,:] = \
                    self.Bound['Linkage']['LAR'][0,:]  
            
            if all(self.Bound['Linkage']['UAR'][1] == 0):
                self.nodes['X']['PoI']['UAR']['Position'][1] = \
                   self.nodes['X']['PoI']['UAF']['Position'][1]

                self.Bound['Linkage']['UAR'][1,:] = \
                    self.Bound['Linkage']['UAF'][1,:]  

    ## Kinematic Design Rules
    # # 1. Nominal Instant Centers # #
    # Front View Swing Arm Length (FVSA)
    self.Target['Linkage']['FVSA'] = 1 \
        / np.arctan(np.abs(self.Target['Linkage']['CamberGain']))

    # Roll Center
    self.nodes['I']['PoI']['RC']['Position'][0] = self.nodes['T']['Position'][0]
    self.nodes['I']['PoI']['RC']['Position'][2] = self.Target['Vehicle']['CG'] \
        * self.Target['Linkage']['RollCenter']/100

    # Front Instant Center
    
    a=1

# %% System Solver