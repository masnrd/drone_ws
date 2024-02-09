import { React, useEffect, useState } from "react";
import { MapContainer, TileLayer, useMapEvents, GeoJSON, LayersControl, FeatureGroup } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { polygonToCells, cellToBoundary } from "h3-js";
import "./Map.css";
import DroneMarker from "./DroneMarker";

export default function Map({ drones, setMap }) {
  const start_position = [1.3430293739520736, 103.9591294705276];
  const [hexagons, setHexagons] = useState([]);

  function H3Overlay() {
    const map = useMapEvents({
      dragend: () => updateHexagons(),
      zoomend: () => updateHexagons(),
    });

    const updateHexagons = () => {
      const bounds = map.getBounds();
      const sw = bounds.getSouthWest();
      const ne = bounds.getNorthEast();
      const expansionAmount = 0.005; // Adjust this value as needed

      // Reference: https://github.com/matthiasfeist/what-the-h3index
      const boundingBox = [
        [sw.lng - expansionAmount, sw.lat - expansionAmount], // Expanded SW corner
        [ne.lng + expansionAmount, sw.lat - expansionAmount], // Bottom-right corner
        [ne.lng + expansionAmount, ne.lat + expansionAmount], // Expanded NE corner
        [sw.lng - expansionAmount, ne.lat + expansionAmount], // Top-left corner
        [sw.lng - expansionAmount, sw.lat - expansionAmount], // Closing the loop (back to expanded SW corner)
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
    <>
      <MapContainer
        center={start_position}
        zoom={18}
        minZoom={16}
        maxZoom={30}
        scrollWheelZoom={true}
        ref={setMap}
      >
        <TileLayer
          zoom={18}
          maxZoom={30}
          attribution="&copy; OpenStreetMap contributors"
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <LayersControl>
          <LayersControl.Overlay name="H3 Overlay">
            <FeatureGroup>
              <H3Overlay />
              <GeoJSON
                key={hexagons.length}
                data={{ type: "FeatureCollection", features: hexagons }}
              />
            </FeatureGroup>
          </LayersControl.Overlay>
      </LayersControl>
        {drones.filter((drone) => (drone.position.lat != null && drone.position.lon != null)).map((drone) => (
          <DroneMarker
            key={drone.drone_id}
            drone_id={drone.drone_id}
            battery_percentage={drone.battery_percentage}
            lon={drone.position.lon}
            lat={drone.position.lat}
            mode={drone.mode}
          ></DroneMarker>
        ))}
      </MapContainer>
    </>
  );
}
