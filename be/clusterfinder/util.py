import folium


def visualize_clusters(clusters):
    # Create a base map
    # Using a sample location
    m = folium.Map(location=[1.3409742208234072,
                   103.9640548435533], zoom_start=15)

    # Define cluster colors (you can expand this list for more clusters)
    colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'lightred', 'beige', 'darkblue',
              'darkgreen', 'cadetblue', 'darkpurple', 'pink', 'lightblue', 'lightgreen', 'gray', 'black', 'lightgray']

    # Add points to the map
    for cluster_id, cluster in clusters.items():
        for point in cluster:
            color = colors[(cluster_id - 1) % len(colors)]
            folium.Marker(
                location=point.coordinates,
                icon=folium.Icon(color=color),
                popup=f"ID:{point.id}, Cluster: {cluster_id}"
            ).add_to(m)

    return m
