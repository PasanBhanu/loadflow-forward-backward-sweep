import numpy
import math
import time
from openpyxl import load_workbook
from Functions import *
from LineModels import *
from DG import *
from LoadModels import *

# Global Variables - Symmetrical Components
a = complex(-0.5, math.sqrt(3)/2)
a_2 = complex(-0.5, -1 * math.sqrt(3)/2)
nodeThreePhaseVoltagePU = numpy.array([[1],[a_2],[a]])
converged = False

# Time
startTime = time.time()

# Load data from excel sheet
wb = load_workbook(filename = 'data.xlsx')
edgesSheet = wb['edges']
nodesSheet = wb['nodes']
voltagesSheet = wb['voltages']
settingsSheet = wb['settings']
sweepOutputSheet = wb['fbsweep']

# Add data to arrays
edges = []
for value in edgesSheet.iter_rows(min_row=2,min_col=1,max_col=5,values_only=True):
    edges.append(value)

nodes = []
for value in nodesSheet.iter_rows(min_row=2,min_col=1,max_col=4,values_only=True):
    nodes.append(value)

voltages = []
for value in voltagesSheet.iter_rows(min_row=2,min_col=1,max_col=3,values_only=True):
    voltages.append(value)

settings = []
for value in settingsSheet.iter_rows(min_row=1,min_col=1,max_col=2,values_only=True):
    settings.append(value)

# Network Limits
startNode = 1
lastNode = maxValueFromExcelColumns(edges, 2)

# Clean Workbook
wb.remove(sweepOutputSheet)
sweepOutputSheet = wb.create_sheet('fbsweep')
sweepOutputSheet['A1'] = 'Nodes'
for row in range(2,lastNode + 2):
    sweepOutputSheet.cell(column=1, row=row, value=row-1)

# Calculate Bases
bases = []
# Voltages for Each Node (Initialisation on Calculated Voltages Array)
calculatedVoltages = []

rating = searchArray(voltages, 1)[2]
voltage = complex(searchArray(voltages, 1)[1])

def calculateBase(startNode, voltage, rating):
    connectedNodes = getConnectedNodes_BS(edges, startNode)
    for connectedNode in connectedNodes:
        if connectedNode[3] == 'T':
            transformerData = searchDoubleArray(edges, startNode, connectedNode[2])[4].split(",")
            rating = float(transformerData[0])
            voltage = complex(transformerData[2])
        bases.append([connectedNode[2], voltage, rating])
        calculateBase(connectedNode[2], voltage, rating)
        # Set Voltage for Each Node
        calculatedVoltages.append([connectedNode[2], nodeThreePhaseVoltagePU])

bases.append([1, voltage, rating])
calculatedVoltages.append([1, nodeThreePhaseVoltagePU])
calculateBase(1, voltage, rating)

# Network iteration
iteration = 1
tolerance = settings[0][1]
maxIterations = settings[1][1]
roundFactor = settings[2][1]

#print ('Bases : ', bases)
#print ('Initial Voltage : ', calculatedVoltages)

# Comparison
calculatedVoltagesOfPreviousIteration = []

### LOOP ###

while (iteration <= maxIterations and converged == False):
    print ('Iteration ', iteration)

    # Variables for Sweep
    calculatedCurrents = []
    calculatedEdgeCurrents = []

    calculatedTotalCurrentsAtNode = [] # Used for algorithmic purpose only

    # Calculate Node Currents
    for node in nodes:
        nodeId = node[0]
        nodeConnection = node[1]
        nodeType = node[2]
        voltage = searchArray(calculatedVoltages, nodeId)[1]
        
        #print ('Node Voltage : ', printMatrix(voltage))
        
        nodeCurrent = numpy.array([[complex(0,0)],[complex(0,0)],[complex(0,0)]])
        if nodeType == 'L':
            # Power (kVA), CPQ, CC, CI
            loadParameters = node[3].split(",")
            base = searchArray(bases, nodeId)
            pu_Power = numpy.array([[complex(loadParameters[0])/base[2]],[complex(loadParameters[1])/base[2]],[complex(loadParameters[2])/base[2]]])

            if nodeConnection == 'S':
                # LN Voltage is Required
                nodeCurrent = loadCurrentStar(voltage, pu_Power, float(loadParameters[3]), float(loadParameters[4]), float(loadParameters[5]))
            else:
                nodeCurrent = loadCurrentDelta(voltage, pu_Power, float(loadParameters[3]), float(loadParameters[4]), float(loadParameters[5]))

        elif nodeType == 'C':
            # kV, kVar
            loadParameters = node[3].split(",")
            base = searchArray(bases, nodeId)
            # Zbase = V_2 / kVA
            z_base = numpy.power((base[1]),2) / (base[2] * 1000)
            kV = numpy.array([[complex(loadParameters[0])],[complex(loadParameters[1])],[complex(loadParameters[2])]])
            kVar = numpy.array([[complex(loadParameters[3])],[complex(loadParameters[4])],[complex(loadParameters[5])]])
            if nodeConnection == 'S':
                nodeCurrent = shuntCapacitanceStar(voltage, kV, kVar, z_base)
            else:
                nodeCurrent = shuntCapacitanceDelta(voltage, kV, kVar, z_base)
        elif nodeType == 'D':
            loadParameters = node[3].split(",")
            base = searchArray(bases, nodeId)
            pu_Power = numpy.array([[complex(loadParameters[0])/base[2]],[complex(loadParameters[1])/base[2]],[complex(loadParameters[2])/base[2]]])
            powerFactor = numpy.array([[float(loadParameters[3])],[float(loadParameters[4])],[float(loadParameters[5])]])
            if nodeConnection == 'S':
                nodeCurrent = generatorStar(voltage, pu_Power, powerFactor)
            else:
                nodeCurrent = generatorStarDelta(voltage, pu_Power, powerFactor)

        #print (nodeId, printMatrixPolar(nodeCurrent * 21.65))
        
        # Add Total Nodal Current to Array
        totalNodeCurrent = searchArray(calculatedCurrents, nodeId)
        if (totalNodeCurrent == 0):
            calculatedCurrents.append([nodeId, nodeCurrent])
        else:
            calculatedCurrents.remove(totalNodeCurrent)
            calculatedCurrents.append([nodeId, numpy.add(totalNodeCurrent[1],nodeCurrent)])

    #print (calculatedCurrents)

    # Clear Voltages Array
    calculatedVoltages = []

    # Backward Sweep
    for i in range(lastNode,0, -1):
        connectedNodes = getConnectedNodes_BS(edges, i)
        destinationNode = getDestinationNode_BS(edges, i)

        # SIGN => (Node ID, Current Matrix)
        current = searchArray(calculatedCurrents,i)
        if current == 0:
            totalCurrentAtNode = [i,numpy.array([[complex(0,0)],[complex(0,0)],[complex(0,0)]])]
        else:
            totalCurrentAtNode = current[1]

        for connectedNode in connectedNodes:
            if (connectedNode[3] == 'L'):
                # SIGN => (Node ID, Current Matrix)
                connectedNodeTotalCurrent = searchArray(calculatedTotalCurrentsAtNode,connectedNode[2])
                totalCurrentAtNode = numpy.add(totalCurrentAtNode, connectedNodeTotalCurrent[1])
            else:
                # Do TF Convert
                print ('TF')
            
        calculatedTotalCurrentsAtNode.append([i, totalCurrentAtNode])

        # print(i, printMatrixPolar ((totalCurrentAtNode * 21.7)))

        if destinationNode != 0 and destinationNode[3] == 'L':
            calculatedEdgeCurrents.append([destinationNode[0], totalCurrentAtNode, i])

    # Forward Sweep
    for i in range(1, lastNode + 1, 1):
        edgeData = getPreviousNode_FS(edges, i)
        if (edgeData == 0):
            newVoltage = nodeThreePhaseVoltagePU
        else:
            if (edgeData[3] == 'L'):
                # SIGN => (Node ID, Voltage Matrix)
                previousNodeVoltage = searchArray(calculatedVoltages,edgeData[0])
                # SIGN => (Start Node ID, Current Matrix, End Node ID)
                edgeCurrent = searchDoubleArray(calculatedEdgeCurrents, edgeData[0], i)
                # Convert Impedance to PU
                base = searchArray(bases, i)
                # Zbase = V_2 / kVA
                z_base = numpy.power((base[1]),2) / (base[2] * 1000)
                # Zero Sequence Impedance, Positive Sequence Impedance
                edgeParameters = edgeData[1].split(",")
                newVoltage = approximateLineModel(previousNodeVoltage[1], edgeCurrent[1] , complex(edgeParameters[0])/z_base, complex(edgeParameters[1])/z_base)
            else:
                print ('TF')

        calculatedVoltages.append([i, newVoltage])
        #print(i, printMatrixPolar ((newVoltage * 400)))

    #print (calculatedVoltages)

    ## Add data to excel
    sweepOutputSheet.cell(column=iteration + 1, row=1, value='Sweep ' + str(iteration))
    for row in range(2,lastNode + 2):
        printVoltage = searchArray(calculatedVoltages,row-1)[1]
        base = searchArray(bases, row-1)
        polarVoltages = []
        for complexPUVoltage in printVoltage:
            polarVoltages.append(convertToPolar(complexPUVoltage[0] * base[1], roundFactor))
        print (", ".join(polarVoltages))
        sweepOutputSheet.cell(column=iteration + 1, row=row, value=str(", ".join(polarVoltages)))

    wb.save('data.xlsx')

    # Check Error
    if iteration > 1:
        totalMaximumError = 0
        for i in range(1, lastNode + 1, 1):
            lastIterationVoltage = searchArray(calculatedVoltagesOfPreviousIteration,i)[1]
            newIterationVoltage = searchArray(calculatedVoltages,i)[1]
            base = searchArray(bases, i)
            errorMatrixPU = numpy.absolute(numpy.subtract(lastIterationVoltage, newIterationVoltage))
            errorMatrix = numpy.multiply(errorMatrixPU, base[1])
            maximumError = numpy.max(errorMatrix)

            if totalMaximumError < maximumError:
                totalMaximumError = maximumError

        if totalMaximumError <= tolerance:
            print ('Converged')
            converged = True

    # Next iteration
    iteration = iteration + 1
    calculatedVoltagesOfPreviousIteration = calculatedVoltages

# Finish Algorithm
endTime = time.time()
print ('Calculation Completed! Elapsed Time : ', endTime - startTime)