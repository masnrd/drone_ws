import React, { useState, useEffect } from "react";

import { MapContainer, TileLayer, GeoJSON } from "react-leaflet";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import DroneMarker from "./DroneMarker.js";

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

  const geojsonObject = {
    type: "FeatureCollection",
    crs: {
      type: "name",
      properties: {
        name: "EPSG:3857",
      },
    },
    features: [
      {
        type: "Feature",
        geometry: {
          type: "MultiPolygon",
          coordinates: [
            [
              [
                [-0.09, 51.505],
                [-0.09, 51.59],
                [-0.12, 51.59],
                [-0.12, 51.505],
              ],
            ],
            [
              [
                [-0.09, 51.305],
                [-0.09, 51.39],
                [-0.12, 51.39],
                [-0.12, 51.305],
              ],
            ],
          ],
        },
      },
    ],
  };

  return (
    <MapContainer center={center_coord} zoom={13}>
      <TileLayer
        attribution='&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      <GeoJSON
        // ref="geojson"
        data={geojsonObject}
        // onEachFeature={this.onEachFeature}
      />{" "}
      {drones.map((drone) => (
        <>
          {" "}
          {/* TODO: Fix this custom icon: https://medium.com/@lillianoquendo1/creating-a-leaflet-map-with-custom-icons-in-a-react-application-c03c74de9edf */}
          <DroneMarker
            drone_id={drone.drone_id}
            battery_percentage={drone.battery_percentage}
            lon={drone.lon}
            lat={drone.lat}
            mode={drone.mode}
          />
        </>
      ))}
    </MapContainer>
  );
}

export default Map;
