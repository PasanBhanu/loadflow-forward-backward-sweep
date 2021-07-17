import numpy
import cmath
import math

# Calculate Primary Currents of YgD Transformer - Backward Sweep (Is & Impedance of T/F)
def backwardTfYgD(Vp, Is, Z, angle):
    a = complex(math.cos((2*math.pi)/3), math.sin((2*math.pi)/3))
    a_2 = a * a
    A = numpy.array([[1,1,1],[1,a_2,a],[1,a,a_2]])
    # Find sequence components (A_inv . Is)
    Is_sequential = numpy.dot(numpy.linalg.inv(A),Is)
    # Apply phase shift
    Is_sequential = Is_sequential * complex(math.cos(angle), math.sin(angle))
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
def fowardTfYgD(Vp, Vs, Is, Z, angle):
    a = complex(math.cos((2*math.pi)/3), math.sin((2*math.pi)/3))
    a_2 = a * a
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
    Vs_sequential = Vs_sequential * complex(math.cos(angle), math.sin(angle))
    Vs = numpy.dot(A, Vs_sequential)

    return Vs

# Calculate Primary Currents of DGy Transformer - Backward Sweep (Is & Impedance of T/F)
def backwardTfDYg(Is, angle):
    a = complex(math.cos((2*math.pi)/3), math.sin((2*math.pi)/3))
    a_2 = a * a
    A = numpy.array([[1,1,1],[1,a_2,a],[1,a,a_2]])
    # Find sequence components (A_inv . Is)
    Is_sequential = numpy.dot(numpy.linalg.inv(A),Is)
    # Save Is_sequential(0) = Is( 
    Is_sequential_0 = Is_sequential[0]
    # Apply phase shift
    Is_sequential = Is_sequential * complex(math.cos(angle), math.sin(angle))
    # Set zero sequnce current to 0
    Is_sequential[0] = 0
    # Calculate primary current
    Ip_sequential = numpy.array([0,Is_sequential[1],Is_sequential[2]])
    Ip = numpy.dot(A, Ip_sequential)

    return Ip

# Calculate Secondary Voltages of YgD Transformer - Foward Sweep (
def fowardTfDYg(Vp, Vs, Is, Z, angle):
    a = complex(math.cos((2*math.pi)/3), math.sin((2*math.pi)/3))
    a_2 = a * a
    A = numpy.array([[1,1,1],[1,a_2,a],[1,a,a_2]])
    # Find sequence components (A_inv . Is)
    Is_sequential = numpy.dot(numpy.linalg.inv(A),Is)
    # Find sequence components (A_inv . Vp)
    Vp_sequential = numpy.dot(numpy.linalg.inv(A),Vp)
    Vs_sequential = numpy.dot(numpy.linalg.inv(A),Vs)
    # Set zero sequence voltage to zero
    Vp_sequential[0] = 0
    # Calculate Vs_sequential
    Vs_sequential = Vp_sequential - numpy.dot([[0,0,0],[0,Z,0],[0,0,Z]], Is_sequential) - numpy.array([Is[0] * Z,[0],[0]])
    # Apply phase shift
    Vs_sequential = Vs_sequential * complex(math.cos(angle), math.sin(angle))
    Vs = numpy.dot(A, Vs_sequential)

    return Vs