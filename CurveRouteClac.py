from math import *

def findCurveRoute(canDistance , trackLength):  #Distance from outer can to car (Meter), track length(Meter)

    AD = 1 - canDistance
    BD = trackLength / 2

    ang_DAB = atan(BD / AD)
    AB = sqrt(BD ** 2 + AD ** 2)
    ang_BCA = pi - 2 * ang_DAB

    AC = (AB * sin(ang_DAB)) / sin(ang_BCA)
    ang_BCE = 2 * ang_BCA

    ang_ACE = ang_BCA

    ang_DEC = pi / 2 - ang_ACE
    starting_angle = pi / 2 - ang_DEC

    return round(AC*1000,2), round(degrees(ang_BCE), 2), round(degrees(starting_angle), 2) # R, theta, start angle


if __name__ == "__main__":
    while True:
        v = float(input(">"))
        print(findCurveRoute(0.2,v))
    