import numpy
import math

def approximateLineModel(voltage, current, zero_impedance, positive_impedance):
    a = complex(-0.5, math.sqrt(3)/2)
    a_2 = complex(-0.5, -1 * math.sqrt(3)/2)
    A = numpy.array([[1,1,1],[1,a_2,a],[1,a,a_2]])
    Z_sequential = numpy.array([[zero_impedance,0,0],[0,positive_impedance,0],[0,0,positive_impedance]])
    Z_approx = numpy.dot(numpy.dot(A,Z_sequential), numpy.linalg.inv(A))
    V = voltage - (1/3) * numpy.dot(Z_approx,current)
    return V