import cmath
import numpy

def maxValueFromExcelColumns(rows, column):
    maxValue = 0
    for row in rows:
        if (row[column] > maxValue):
            maxValue = row[column]
    return maxValue

# Array : [array, value]
def searchArray(array, value):
    for row in array:
        if (row[0] == value):
            return row
    return 0

# Array : [array, from, to]
def searchDoubleArray(arr, start, end):
    for row in arr:
        if (row[0] == start and row[2] == end):
            return row
    return 0

def printMatrix(matrix):
    s = [[str(e) for e in row] for row in matrix]
    lens = [max(map(len, col)) for col in zip(*s)]
    fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
    table = [fmt.format(*row) for row in s]
    return '\n'.join(table)

def printMatrixPolar(matrix):
    s = [[str(str(abs(e)) + ' ∠ ' + str(numpy.degrees(cmath.phase(e)))) for e in row] for row in matrix]
    lens = [max(map(len, col)) for col in zip(*s)]
    fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
    table = [fmt.format(*row) for row in s]
    return '\n'.join(table)

# Convert cartisan to polar
def convertToPolar(value, roundFactor):
    polarStr = str(round(abs(value),roundFactor)) + ' ∠ ' + str(round(numpy.degrees(cmath.phase(value)),roundFactor))
    return polarStr

def getPolarMagnitude(value, roundFactor):
    polarMagnitude = round(abs(value),roundFactor)
    return polarMagnitude

# Convert polar to cartisan form (Value, Angle in Degrees)
def convertToCartisan(value, angle):
    return complex(value * numpy.cos(angle),value * numpy.sin(angle))

# ---- Backward Sweep ---- #

# Get destination of the given edge
def getDestinationNodeId_BS(arr, node):
    for row in arr:
        if (row[2] == node):
            return row[0]
    return 0

def getDestinationNode_BS(arr, node):
    for row in arr:
        if (row[2] == node):
            return row
    return 0

# Get previously connected nodes for a given node
def getConnectedNodesId_BS(arr, node):
    connectedNodes = []
    for row in arr:
        if (row[0] == node):
            connectedNodes.append(row[2])
    return connectedNodes

def getConnectedNodes_BS(arr, node):
    connectedNodes = []
    for row in arr:
        if (row[0] == node):
            connectedNodes.append(row)
    return connectedNodes


def getCalculatedTotalCurrentsAtNode(arr, node):
    for row in arr:
        if (row[0] == node):
            return row[1]
    return 0

# ---- Foward Sweep ---- #

# Get previously connected nodes for a given node
def getPreviousNodeId_FS(arr, node):
    for row in arr:
        if (row[2] == node):
            return row[0]
    return 0

def getPreviousNode_FS(arr, node):
    for row in arr:
        if (row[2] == node):
            return row
    return 0
