import React, { useState, useEffect } from "react";

import { MapContainer, TileLayer, Marker } from "react-leaflet";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import markerIconPng from "leaflet/dist/images/marker-icon.png";

function Map() {
  var center_coord = [1.3399775009363866, 103.96258672159254];

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

  const droneIcon = L.icon({
    iconUrl: markerIconPng,
  });

  return (
    <MapContainer center={center_coord} zoom={13}>
      <TileLayer
        attribution='&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      {drones.map((drone) => (
        <Marker
          key={drone.drone_id}
          position={[drone.lat, drone.lon]}
          icon={droneIcon}
        />
      ))}
    </MapContainer>
  );
}

export default Map;
