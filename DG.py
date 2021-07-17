import numpy
import math

# --------------------------------- #
# ----- DG With Complex Power ----- #
# --------------------------------- #

# Line Current of Star Connected DG - Voltage (LN)
def generatorStar(voltage, power):
    I = numpy.divide(power,voltage)
    return I

# Line Current of Delta Connected DG - Voltage (LL)
def generatorStarDelta(voltage, power):
    I = numpy.divide(power,voltage)
    return I

# --------------------------------- #
# ----- DGs With Power Factor ----- #
# --------------------------------- #

# Line Current of Star Connected DG - Voltage (LN)
def generatorStar(voltage, power, powerFactor):
    Q = power * math.tan(math.acos(powerFactor))
    I = numpy.divide(complex(power,Q),voltage)

    return I

# Line Current of Delta Connected DG - Voltage (LL)
def generatorStarDelta(voltage, power, powerFactor):
    Q = power * math.tan(math.acos(powerFactor))
    I = numpy.divide(complex(power,Q),voltage)

    return I

    # Transformation Matrix is not required since we do the calculation either on LN or LL.
    
    # tfMatrix = numpy.array([[1,0,-1],[-1,1,0],[0,-1,1]])
    # return numpy.dot(tfMatrix, I)