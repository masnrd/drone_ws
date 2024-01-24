import geopy.distance


class Point:
    def __init__(self, id, coordinates):
        self.id = id
        self.coordinates = coordinates
        self.visited = False
        self.cluster = None
        self.is_noise = False

    def distance(self, other_point):
        """
        Compute the Euclidean distance between this point and another point.
        """
        return geopy.distance.geodesic(self.coordinates, other_point.coordinates).km

    def __str__(self):
        return f'''Point {self.id}[{str(self.coordinates)}, cluster:{self.cluster}]'''
