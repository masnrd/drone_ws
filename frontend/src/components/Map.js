import { React, useState } from "react";
import {
  MapContainer,
  TileLayer,
  useMapEvents,
  Marker,
  Popup,
  Circle,
  Polyline,
  // LayersControl, FeatureGroup, GeoJSON,
} from "react-leaflet";
import "leaflet/dist/leaflet.css";
// import {polygonToCells, cellToBoundary} from "h3-js";
import "./Map.css";
import Icons from "./Icon";
import FakeDetection from "./FakeDetection";

export default function Map({
  drones,
  hotspots,
  clusters,
  detectedEntities,
  setMap,
}) {
  const start_position = [1.3399775009363866, 103.96258672159254];

  const addHotspot = (latlng) => {
    const url = "http://127.0.0.1:5000/hotspot/add";
    const params = new URLSearchParams();
    params.append("hotspot_position", JSON.stringify({ latlng }));
    fetch(url, { method: "POST", body: params });
  };

  return (
    <>
      <MapContainer
        center={start_position}
        zoom={18}
        minZoom={16}
        maxZoom={30}
        scrollWheelZoom={true}
        ref={setMap}>
        <TileLayer
          zoom={18}
          maxZoom={30}
          attribution="&copy; OpenStreetMap contributors"
          url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
          // url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <AddHotspotOnClick onNewPoint={addHotspot} />
        {drones
          .filter(
            (drone) => drone.position.lat != null && drone.position.lon != null
          )
          .map((drone) => (
            <>
              <Marker
                key={drone.drone_id}
                position={{ lat: drone.position.lat, lng: drone.position.lon }}
                icon={Icons.createDroneIcon(drone.drone_id)}>
                <Popup>
                  Drone ID: {drone.drone_id}
                  <br />
                  Battery: {drone.battery_percentage.toFixed(1)}%<br />
                  Mode: {drone.mode}
                </Popup>
              </Marker>
              {drone.simulated_path && (
                <Polyline
                  positions={Object.values(drone.simulated_path).map(
                    (point) => [point.lat, point.lon]
                  )}
                  color="black"
                  weight={3}
                />
              )}
            </>
          ))}
        {hotspots.map((hotspot, index) => (
          <Marker
            key={index}
            position={{ lat: hotspot[0], lng: hotspot[1] }}
            icon={Icons.createHotSpotIcon()}>
            <Popup>{`(${hotspot[0]}, ${hotspot[1]})`}</Popup>
          </Marker>
        ))}
        {clusters.map((cluster, index) => (
          <div key={`marker-${index}`}>
            <Marker
              key={`marker-${index}`}
              position={{ lat: cluster[0][0], lng: cluster[0][1] }}
              icon={Icons.createClusterIcon(index + 1)}>
              <Popup>{`(${cluster[0]}, ${cluster[0]})`}</Popup>
            </Marker>
            <Circle
              key={`circle-${index}`}
              center={{ lat: cluster[0][0], lng: cluster[0][1] }}
              radius={30}
              color="#ffd214"
              fillColor="#ffd214"
              fillOpacity={0.4}
              weight={0}
            />
          </div>
        ))}
        {detectedEntities
          .filter((entity) =>
            FakeDetection.shouldDisplayEntity(entity, clusters, 0.3)
          )
          .map((entity, index) => (
            <Marker
              key={`entity-marker-${index}`}
              position={{
                lat: entity.coordinates.lat,
                lng: entity.coordinates.lon,
              }}
              icon={Icons.createDetectionIcon()}></Marker>
          ))}
      </MapContainer>
    </>
  );

  function AddHotspotOnClick({ onNewPoint }) {
    useMapEvents({
      click(e) {
        onNewPoint(e.latlng);
      },
    });
    return null;
  }
}
