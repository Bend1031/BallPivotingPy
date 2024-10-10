from .Enums import EdgeType
from .utilities import *


class Edge:

    def __init__(self, source, target):
        self.source = source
        self.target = target
        self.facet1 = None
        self.facet2 = None
        self.type = None

        if source is not None and target is not None:
            self.source.addAdjacentEdge(self)
            self.target.addAdjacentEdge(self)
            self.setType(EdgeType.FRONT)

    def getSource(self):
        return self.source

    def getTarget(self):
        return self.target

    def getFacet1(self):
        return self.facet1

    def getFacet2(self):
        return self.facet2

    def addAdjacentFacet(self, facet):
        if self.facet1 == facet or self.facet2 == facet:
            return False

        if self.facet1 is None:
            self.facet1 = facet
            self.updateOrientation()
            self.setType(EdgeType.FRONT)
            return True

        if self.facet2 is None:
            self.facet2 = facet
            self.setType(EdgeType.INNER)
            return True

        print("Already two triangles")
        return False

    def removeAdjacentFacet(self, facet):
        if self.facet1 == facet:

            self.facet1 = None
            self.setType(EdgeType.BORDER)
            return True

        if self.facet2 == facet:

            self.facet2 = None
            self.setType(EdgeType.BORDER)
            return True

        return False

    def updateOrientation(self):
        opp = self.getOppositeVertex()
        if opp is None:
            return
        v = [
            self.target.x - self.source.x,
            self.target.y - self.source.y,
            self.target.z - self.source.z,
        ]
        w = [opp.x - self.source.x, opp.y - self.source.y, opp.z - self.source.z]
        vw = cross_product(v, w)
        nv = normalize(vw)

        nx, ny, nz = (
            self.source.nx + self.target.nx + opp.nx,
            self.source.ny + self.target.ny + opp.ny,
            self.source.nz + self.target.nz + opp.nz,
        )
        nx, ny, nz = normalize([nx, ny, nz])

        if dot(nv, [nx, ny, nz]) < 0:
            self.source, self.target = self.target, self.source

    def hasVertex(self, vertex):
        return vertex == self.source or vertex == self.target

    def isInnerEdge(self):
        if self.facet2 is None:
            return False
        return True

    def getType(self):
        return self.type

    def setType(self, type):
        self.type = type

    def getOppositeVertex(self):
        if self.facet1 is None:
            return None

        vertices = self.facet1.vertexes
        for vertex in vertices:
            if vertex is not self.source and vertex is not self.target:
                return vertex
        return None
