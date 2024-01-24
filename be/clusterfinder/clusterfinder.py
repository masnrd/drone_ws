from .interface import ClusterFinder

# DBScan ClusterFinder


class DBSCANClusterFinder(ClusterFinder):
    """
    DBSCAN Clustering Object using the DBSCAN algorithm.

    Args:
    - max_gap (int): The maximum distance (in km) between two points to be considered neighbors.
    - min_pts (int): The minimum number of neighbors required to be considered a core point.
    """

    def __init__(self, dataset, max_gap=1, min_pts=2):
        super().__init__(dataset)
        self.max_gap = max_gap
        self.min_pts = min_pts

    def region_query(self, point):
        """
        Returns all points within eps-distance from the given point in the dataset.
        """
        neighbors = []
        for candidate_point in self.dataset:
            if point.distance(candidate_point) < self.max_gap:
                neighbors.append(candidate_point)
        return neighbors

    def expand_cluster(self, point, neighbors):
        """
        Expands the cluster of the given point and its neighbors.
        """
        point.cluster = self.cluster_count
        self.clusters[self.cluster_count] = [point]
        i = 0
        while i < len(neighbors):
            neighbor = neighbors[i]
            if not neighbor.visited:
                neighbor.visited = True
                new_neighbors = self.region_query(neighbor)
                if len(new_neighbors) >= self.min_pts:
                    neighbors += new_neighbors
            if neighbor.cluster is None:
                neighbor.cluster = self.cluster_count
                self.clusters[self.cluster_count].append(neighbor)
            i += 1

    def fit(self):
        """
        Run the DBSCAN clustering algorithm.
        """
        for point in self.dataset:
            if not point.visited:
                point.visited = True
                neighbors = self.region_query(point)
                if len(neighbors) < self.min_pts:
                    point.is_noise = True
                else:
                    self.cluster_count += 1
                    self.expand_cluster(point, neighbors)
        return self.clusters


# DIANA ClusterFinder


class DIANA_Cluster:
    def __init__(self, points):
        self.points = points

    def longest_distance(self):
        """
        Finds the maximum distance between two points in a cluster and returns the two points
        """
        max_distance = 0
        A, B = None, None
        for i in range(len(self.points)):
            for j in range(i+1, len(self.points)):
                d = self.points[i].distance(self.points[j])
                if d > max_distance:
                    max_distance = d
                    A, B = self.points[i], self.points[j]
        return max_distance, [A, B]

    def split(self, A, B):
        # Find two points with maximum dissimilarity
        cluster_a = [A]
        cluster_b = [B]

        for i in range(len(self.points)):
            if self.points[i] in (A, B):
                pass
            else:
                if self.points[i].distance(A) > self.points[i].distance(B):
                    cluster_b.append(self.points[i])
                else:
                    cluster_a.append(self.points[i])

        return DIANA_Cluster(cluster_a), DIANA_Cluster(cluster_b)

    def __str__(self):
        return f'''DIANA_Cluster:{[i.id for i in self.points]}'''


class DIANAClusterFinder(ClusterFinder):
    """
    DIANA Clustering Object using the DIANA algorithm.

    Args:
    - threshold (int): The maximum distance between two points in a cluster in km.
    """

    def __init__(self, dataset, threshold=1.0):
        super().__init__(dataset)
        self.threshold = threshold

    def fit(self):
        initial_cluster = DIANA_Cluster(self.dataset)

        output_clusters = []
        clusters_to_split = [initial_cluster]
        while len(clusters_to_split) > 0:
            target_cluster = clusters_to_split.pop()
            max_distance, furthest_pts = target_cluster.longest_distance()
            if max_distance > self.threshold:
                clusters_to_split += target_cluster.split(
                    furthest_pts[0], furthest_pts[1])
            else:
                output_clusters.append(target_cluster)

        for i in range(len(output_clusters)):
            self.clusters[i] = output_clusters[i].points

        return self.clusters
