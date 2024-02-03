import { React, useEffect, useState } from "react";
import { MapContainer, TileLayer, useMapEvents, GeoJSON } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { polygonToCells, cellToBoundary } from "h3-js";
import "./Map.css";
import DroneMarker from "./DroneMarker";
import DroneDrawer from "./DroneDrawer";
import Fab from "@mui/material/Fab";
import AddIcon from "@mui/icons-material/Add";

export default function Map() {
  const start_position = [1.3430293739520736, 103.9591294705276];
  const [hexagons, setHexagons] = useState([]);
  const [drones, setDrones] = useState([]);
  const [selectedDrone, setSelectedDrone] = useState(null);
  const [drawerOpen, setDrawerOpen] = useState(true);

  function handleDroneClick(id) {
    console.log("Set drone", id);
    setSelectedDrone(id);
  }

  const url = "http://127.0.0.1:5000/api/info";
  useEffect(() => {
    fetch(url, { mode: "cors" })
      .then((response) => response.json())
      .then((data) => {
        console.log(data);
        const dronesData = data["drones"];
        const parsedDrones = Object.values(dronesData);
        setDrones(parsedDrones);
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
      {drones ? (
        <DroneDrawer initialOpen={drawerOpen} droneData={drones} />
      ) : (
        <Fab className="fab-button" color="primary" aria-label="add">
          <AddIcon />
        </Fab>
      )}
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
        <GeoJSON
          key={hexagons.length}
          data={{ type: "FeatureCollection", features: hexagons }}
        />
        {drones.map((drone) => (
          <DroneMarker
            key={drone.drone_id}
            drone_id={drone.drone_id}
            battery_percentage={drone.battery_percentage}
            lon={drone.lon}
            lat={drone.lat}
            mode={drone.mode}
            setSelectedDroneEvent={handleDroneClick} // Changed here
          ></DroneMarker>
        ))}
      </MapContainer>
    </>
  );
}
