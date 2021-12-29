import Latombe as lt
from shapely.geometry import Point, Polygon, LineString
# ======================================================================================================================
'''Defining the test paths and environments'''
# ======================================================================================================================
# Path + Block - Directly taken from example in slides - Origin in middle

path1 = [
    [-5, -5],
    [-4, 1],
    [-5, 5],
    [-2, 5],
    [-1, 3],
    [4, 3],
    [5, -1]
        ]

block = dict()
block["obs1"] = Polygon([(-2, -6), (-2, 2), (2, 2), (2, -6)]).buffer(.5)

test1 = lt.Smoothing(path1, block, 3)

test1.smoother()

# ======================================================================================================================
'''End of File'''
# ======================================================================================================================