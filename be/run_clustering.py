import numpy as np

from clusterfinder.point import Point
from clusterfinder.clusterfinder import DIANAClusterFinder
from typing import List, Dict


def run_clustering(hotspots_location: List[Dict]):
    hotspots = [(entry["lat"], entry["lng"]) for entry in hotspots_location]
    hotspots = [Point(i, hotspots[i]) for i in range(len(hotspots))]

    # Find clusters
    cluster_finder = DIANAClusterFinder(hotspots, threshold=1.0)
    cluster_results = cluster_finder.fit()

    # Sort by most hotspots:
    sorted_clusters = {
        k: v for k, v in sorted(cluster_results.items(), key=lambda item: len(item[1]), reverse=True)
    }
    cluster_results = sorted_clusters

    # Find cluster centers
    all_centres = dict()

    for cluster_id, cluster in cluster_results.items():
        centre = find_search_centre(cluster)
        all_centres[cluster_id] = centre

    return all_centres


def find_search_centre(cluster: list[Point]) -> tuple[float, float]:
    """
    Find the geographic center of a cluster of Points.

    :param cluster: A list of Point objects.
    :return: A Point object representing the geographic center of the cluster.
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

    # Create a new Point object for the center
    centre_point = (central_latitude, central_longitude)

    return centre_point
