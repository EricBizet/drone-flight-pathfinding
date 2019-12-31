import math

class Utils:

    @staticmethod
    def euclidean_distance(position, reference_point):
        #compute heuristic cost
        return math.sqrt(math.pow(position[0] - reference_point[0], 2) + math.pow(position[1] - reference_point[1], 2))

    @staticmethod
    def manhattan_distance(position, reference_point):
        #compute heuristic cost
        return abs(position[0] - reference_point[0]) + abs(position[1] - reference_point[1])

    