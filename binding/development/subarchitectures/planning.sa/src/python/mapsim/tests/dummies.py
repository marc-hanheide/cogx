""" Mock objects to be used when testing other components.

    Mock objects should provide (more or less) the same API as their MAPSIM counterparts.
"""

class Agent(object):
    """ agent dummy"""
    def __init__(self, simulator, name=None):
        self.simulator = simulator
        self.name = name
