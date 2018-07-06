class Robot:
    def __init__(robotDiameter, mapResolution):
        self.robotDiameter = robotDiameter
        self.pixelRobotDiameter = int(robotDiameter / mapResolution)
        # TODO eventually, do this programmatically using the Midpoint Circle
        # algorithm, see :
        # https://stackoverflow.com/questions/35539471/how-to-translate-the-midpoint-circle-algorithm-into-matplotlib
        self.robotFootprint = numpy.array
        self.robotFootprint2x = numpy.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                             [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                                             [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                             [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                             [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                             [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                             [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                             [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                             [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0],
                                             [0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])