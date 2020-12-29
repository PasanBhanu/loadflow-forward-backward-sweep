import numpy
import cmath
import math

# Calculate Primary Currents of YgD Transformer - Backward Sweep (Is & Impedance of T/F)
def backwardTfYgD(Vp, Is, Z):
    a = exp(complex(0,(2*math.pi)/3))
    A = numpy.array([[1,1,1],[1,a^2,a],[1,a,a^2]])
    # Find sequence components (A_inv . Is)
    Is_sequential = numpy.dot(numpy.linalg.inv(A),Is)
    # Apply phase shift
    Is_sequential = Is_sequential * exp(complex(0,math.pi/6))
    # Set zero sequnce current to 0
    Is_sequential[0] = 0
    # Calculate sequence primary voltages
    Vp_sequential = numpy.dot(numpy.linalg.inv(A),Vp)
    # Calculate zero sequence primary current
    Ip_zero = Vp_sequential[0]/Z[0]
    # Calculate primary current
    Ip_sequential = numpy.array([[Ip_zero],[Is_sequential[1]],[Is_sequential[2]]])
    Ip = numpy.dot(A, Ip_sequential)

    return Ip

# Calculate Secondary Voltages of YgD Transformer - Foward Sweep (
def fowardTfYgD(Vp, Is, Z):
    a = exp(complex(0,(2*math.pi)/3))
    A = numpy.array([[1,1,1],[1,a^2,a],[1,a,a^2]])
    # Find sequence components (A_inv . Is)
    Is_sequential = numpy.dot(numpy.linalg.inv(A),Is)
    # Apply phase shift
    Is_sequential = Is_sequential * exp(complex(0,math.pi/6))
    # Set zero sequnce current to 0
    Is_sequential[0] = 0
    # Calculate sequence primary voltages
    Vp_sequential = numpy.dot(numpy.linalg.inv(A),Vp)
    # Calculate zero sequence primary current
    Ip_zero = Vp_sequential[0]/Z_zero
    # Calculate primary current
    Ip_sequential = numpy.array([[Ip_zero],[Is_sequential[1]],[Is_sequential[2]]])
    Ip = numpy.dot(A, Ip_sequential)

    return Ip

    
    
