import numpy as np

from clusterfinder.point import Point
from clusterfinder.clusterfinder import DIANAClusterFinder
from typing import List, Dict


def run_clustering(hotspots_location: List[Dict]):
    hotspots = [(entry["lat"], entry["lng"]) for entry in hotspots_location]
    hotspots = [Point(i, hotspots[i]) for i in range(len(hotspots))]

    # Find clusters
    cluster_finder = DIANAClusterFinder(hotspots, threshold=0.1)
    cluster_results = cluster_finder.fit()

    # Sort by most hotspots:
    sorted_clusters = {
        k: v for k, v in sorted(cluster_results.items(), key=lambda item: len(item[1]), reverse=True)
    }
    cluster_results = sorted_clusters

    # Find cluster centers
    all_centres = dict()

    idx = 0
    for _, cluster in cluster_results.items():
        centre, max_distance = find_search_centre(cluster)
        all_centres[idx] = [centre, max_distance, len(cluster)]
        idx += 1

    return all_centres


def find_search_centre(cluster: list[Point]) -> tuple:
    """
    Find the geographic center of a cluster of Points and the maximum distance from the centre to a point.

    :param cluster: A list of Point objects.
    :return: A tuple containing:
             - A tuple representing the geographic center of the cluster (latitude, longitude).
             - The maximum distance from the centre to a point in the cluster. (in m)
    """

    if not cluster:
        raise ValueError("The cluster is empty")

    # Convert all points to Cartesian coordinates
    x, y, z = 0.0, 0.0, 0.0

    for point in cluster:
        latitude = np.radians(point.coordinates[0])
        longitude = np.radians(point.coordinates[1])

        x += np.cos(latitude) * np.cos(longitude)
        y += np.cos(latitude) * np.sin(longitude)
        z += np.sin(latitude)

    # Compute average coordinates
    total_points = len(cluster)
    x /= total_points
    y /= total_points
    z /= total_points

    # Convert average coordinates back to latitude and longitude
    central_longitude = np.arctan2(y, x)
    central_square_root = np.sqrt(x * x + y * y)
    central_latitude = np.arctan2(z, central_square_root)

    # Convert radians back to degrees
    central_latitude = np.degrees(central_latitude)
    central_longitude = np.degrees(central_longitude)

    # Calculate the maximum distance
    max_distance = 0
    for point in cluster:
        max_distance = max(max_distance, haversine(central_latitude, central_longitude, point.coordinates[0], point.coordinates[1]))

    centre_point = (central_latitude, central_longitude)
    return centre_point, max_distance*1000

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees).
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arcsin(np.sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r
