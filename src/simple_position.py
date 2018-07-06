import numpy

class SimplePosition:
    idCounter = 1
    
    def __init__(self, x, y):
        self.coords = numpy.array([round(x), round(y)])
        self.id = SimplePosition.idCounter
        SimplePosition.idCounter = SimplePosition.idCounter + 1
        
    @classmethod
    def from_array(cls, array):
        return cls(pose[0], pose[1]))
        
   def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(other, SimplePosition):
            return self.coords[0] == other.coords[0] && self.coords[1] == other.coords[1]
        return NotImplemented

    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        x = self.__eq__(other)
        if x is not NotImplemented:
            return not x
        return NotImplemented

    def __hash__(self):
        """Overrides the default implementation"""
        return id