import networkx as nx
import numpy as np
import datetime

class KinematicSystem(nx.Graph):
    def __init__(self, Name=str(), Target=dict(), Bound=dict()):
        self.Date = datetime.dateime.now()

        