import numpy
import cmath
import math

# Calculate Primary Currents of YgD Transformer - Backward Sweep (Is & Impedance of T/F)
def backwardTfYgD(Vp, Is, Z):
    a = complex(-0.5, math.sqrt(3)/2)
    a_2 = complex(-0.5, -1 * math.sqrt(3)/2)
    A = numpy.array([[1,1,1],[1,a_2,a],[1,a,a_2]])
    # Find sequence components (A_inv . Is)
    Is_sequential = numpy.dot(numpy.linalg.inv(A),Is)
    # Apply phase shift
    Is_sequential = Is_sequential * complex(math.sqrt(3)/2,0.5)
    # Set zero sequnce current to 0
    Is_sequential[0] = 0
    # Calculate sequence primary voltages
    Vp_sequential = numpy.dot(numpy.linalg.inv(A),Vp)
    # Calculate zero sequence primary current
    Ip_zero = Vp_sequential[0]/Z
    # Calculate primary current
    Ip_sequential = numpy.array([Ip_zero,Is_sequential[1],Is_sequential[2]])
    Ip = numpy.dot(A, Ip_sequential)

    return Ip

# Calculate Secondary Voltages of YgD Transformer - Foward Sweep (
def fowardTfYgD(Vp, Vs, Is, Z):
    a = complex(-0.5, math.sqrt(3)/2)
    a_2 = complex(-0.5, -1 * math.sqrt(3)/2)
    A = numpy.array([[1,1,1],[1,a_2,a],[1,a,a_2]])
    # Find sequence components (A_inv . Is)
    Is_sequential = numpy.dot(numpy.linalg.inv(A),Is)
    # Find sequence components (A_inv . Vp)
    Vp_sequential = numpy.dot(numpy.linalg.inv(A),Vp)
    Vs_sequential = numpy.dot(numpy.linalg.inv(A),Vs)
    # Set zero sequence voltage to zero
    Vp_sequential[0] = 0
    # Calculate Vs_sequential
    Vs_sequential = Vp_sequential - numpy.dot([[0,0,0],[0,Z,0],[0,0,Z]], Is_sequential) + numpy.array([Vs_sequential[0],[0],[0]])
    # Apply phase shift
    Vs_sequential = Vs_sequential * complex(math.sqrt(3)/2,0.5)
    Vs = numpy.dot(A, Vs_sequential)

    return Vs

    
    
