import numpy

#from Functions import *
#print (printMatrixPolar(total * 138.88))
#print (printMatrix(z * 51.84))

# Line Current of Star Connected Load - Voltage (LN)
def loadCurrentStar(voltage, power, cpq, cc, ci):
    Ipq = cpq * numpy.conjugate(numpy.divide(power,voltage))
    z = numpy.divide(numpy.power(abs(voltage), 2), numpy.conjugate(power))
    Iz = numpy.divide(voltage, z) * ci
    Iz = numpy.nan_to_num(Iz)
    Im = cc * numpy.conjugate(numpy.divide(power,voltage))
    total = numpy.add(Ipq, numpy.add(Iz,Im))
    return total

# Line Current of Delta Connected Load - Voltage (LL)
def loadCurrentDelta(voltage, power, cpq, cc, ci):
    Ipq = cpq * numpy.conjugate(numpy.divide(power,voltage))
    z = numpy.divide(numpy.power(abs(voltage), 2), numpy.conjugate(power))
    Iz = numpy.divide(voltage, z) * ci
    Iz = numpy.nan_to_num(Iz)
    Im = cc * numpy.conjugate(numpy.divide(power,voltage))
    total = numpy.add(Ipq, numpy.add(Iz,Im))
    return total

    # Transformation Matrix is not required since we do the calculation either on LN or LL.
    
    # tfMatrix = numpy.array([[1,0,-1],[-1,1,0],[0,-1,1]])
    # return numpy.dot(tfMatrix, total)

# Line Current of Star Shunt Capacitance - Voltage (LN)
def shuntCapacitanceStar(voltage, kV, kVar, z_base):
    Z = numpy.divide(numpy.power(kV,2), numpy.divide(kVar, 1000))
    Z = numpy.nan_to_num(Z)
    Z_pu = numpy.divide(Z, z_base)
    I = numpy.divide(voltage, Z_pu)
    return I

# Line Current of Delta Shunt Capacitance - Voltage (LL)
def shuntCapacitanceDelta(voltage, kV, kVar, z_base):
    Z = numpy.divide(numpy.power(kV,2), numpy.divide(kVar, 1000))
    Z = numpy.nan_to_num(Z)
    Z_pu = numpy.divide(Z, z_base)
    I = numpy.divide(voltage, Z_pu)
    return I

    # Transformation Matrix is not required since we do the calculation either on LN or LL.
    
    # tfMatrix = numpy.array([[1,0,-1],[-1,1,0],[0,-1,1]])
    # return numpy.dot(tfMatrix, I)