import { React, useEffect, useState } from "react";
import { MapContainer, TileLayer, useMapEvents, GeoJSON } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { polygonToCells, cellToBoundary } from "h3-js";
import "./Map.css";
import DroneMarker from "./DroneMarker";

export default function Map() {
  const start_position = [1.3430293739520736, 103.9591294705276];
  const [hexagons, setHexagons] = useState([]);
  const [drones, setDrones] = useState([]);

  const url = "http://127.0.0.1:5000/api/info";
  useEffect(() => {
    fetch(url, { mode: "cors" })
      .then((response) => response.json())
      .then((data) => {
        const parsedDrones = Object.values(data).map((droneData) =>
          JSON.parse(droneData)
        );
        setDrones(parsedDrones);
        console.log(parsedDrones);
      })
      .catch((error) => console.error("Error in fetching drone data:", error));
  });

  function H3Overlay() {
    const map = useMapEvents({
      dragend: () => updateHexagons(),
      zoomend: () => updateHexagons(),
    });

    const updateHexagons = () => {
      const bounds = map.getBounds();
      const sw = bounds.getSouthWest();
      const ne = bounds.getNorthEast();

      // Reference: https://github.com/matthiasfeist/what-the-h3index
      const boundingBox = [
        [sw.lng, sw.lat],
        [ne.lng, sw.lat],
        [ne.lng, ne.lat],
        [sw.lng, ne.lat],
        [sw.lng, sw.lat], // Closing the loop
      ];

      const h3Resolution = calculateH3Resolution(map.getZoom());

      const hexIndexes = polygonToCells(boundingBox, h3Resolution, true);

      const hexFeatures = hexIndexes.map((index) => {
        const boundary = cellToBoundary(index);
        return {
          type: "Feature",
          properties: {},
          geometry: {
            type: "Polygon",
            coordinates: [boundary.map((coord) => [coord[1], coord[0]])],
          },
        };
      });

      setHexagons(hexFeatures);
    };

    const calculateH3Resolution = (zoom) => {
      // Implement logic to determine resolution based on zoom level
      return 11; // Placeholder value
    };

    return null;
  }

  return (
    <MapContainer
      center={start_position}
      zoom={18}
      minZoom={16}
      maxZoom={30}
      scrollWheelZoom={true}
      // style={{ height: "100vh" }}
    >
      <TileLayer
        zoom={18}
        minZoom={16}
        maxZoom={30}
        attribution="&copy; OpenStreetMap contributors"
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      <H3Overlay />
      {/* Use hexagons length as key */}
      <GeoJSON
        key={hexagons.length}
        data={{ type: "FeatureCollection", features: hexagons }}
      />
      {drones.map((drone) => (
        <DroneMarker
          drone_id={drone.drone_id}
          battery_percentage={drone.battery_percentage}
          lon={drone.lon}
          lat={drone.lat}
          mode={drone.mode}
        />
      ))}
    </MapContainer>
  );
}
