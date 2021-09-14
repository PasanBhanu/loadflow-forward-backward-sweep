import numpy
import math

def approximateLineModel(voltage, current, zero_impedance, positive_impedance):
    # Calculate matrix values
    d = (2 * positive_impedance) + zero_impedance
    o = zero_impedance - positive_impedance
    # Build matrix
    Z_approx = numpy.array([[d,o,o],[o,d,o],[o,o,d]])
    # Calculate voltage
    V = voltage - (1/3) * numpy.dot(Z_approx,current)
    return V

# ToDo: Length of the line is not considered. Add it to calculations
def calculateLineResistance(resistance_p, gmr_p, resistance_n, gmr_n, phases, length, isNeutralAvailable, frequency, soilResistivity, type, l_12, l_13, l_23, l_1n, l_2n, l_3n):
    r_11 = r_22 = r_33 = r_12 = r_13 = r_23 = r_1n = r_2n = r_3n = complex(0,0)

    r_ii = complex(resistance_p + 0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / gmr_p) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))
    r_nn = complex(resistance_n + 0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / gmr_n) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))

    if phases == 3:
        r_12 = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_12) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))
        r_13 = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_13) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))
        r_23 = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_23) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))

        r_11 = r_22 = r_33 = r_ii
    elif phases == 2:
        r_12 = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_12) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))

        r_11 = r_12 = r_ii
    else:
        r_11 = r_ii

    if isNeutralAvailable:
        if phases == 3:
            r_1n = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_1n) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))
            r_2n = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_2n) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))
            r_3n = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_3n) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))
        elif phases == 2:
            r_1n = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_1n) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))
            r_2n = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_2n) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))
        else:
            r_1n = complex(0.00158836 * frequency, 0.00202237 * frequency * (math.log(1 / l_1n) + 7.6786 + 0.5 * math.log(soilResistivity / frequency)))

    z_ij = numpy.array([[r_11, r_12, r_13], [r_12, r_22, r_23], [r_13, r_23, r_33]])
    z_in = numpy.array([[r_1n], [r_2n], [r_3n]])
    z_nn = numpy.array([[r_nn]])
    z_nj = numpy.transpose(z_in)

    z_abc = z_ij - numpy.dot(z_in, numpy.dot(1/z_nn,z_nj))

    a = complex(-0.5, math.sqrt(3)/2)
    a_2 = complex(-0.5, -1 * math.sqrt(3)/2)
    A = numpy.array([[1,1,1],[1,a_2,a],[1,a,a_2]])
    z_sequential = numpy.dot(numpy.linalg.inv(A), numpy.dot(z_abc, A))

    resistance = str(round(z_sequential[0][0],4)) + "," + str(round(z_sequential[1][1],4))
    resistance = resistance.replace('(','').replace(')','')

    return resistance