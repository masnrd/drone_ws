import React, { useState, useEffect } from "react";
import markerIconPng from "leaflet/dist/images/marker-icon.png";
import L from "leaflet";
import { Marker } from "react-leaflet";

function DroneMarker(props) {
  const droneIcon = L.icon({
    iconUrl: markerIconPng,
  });
  return (
    <>
      <Marker
        key={props.drone_id}
        position={[props.lat, props.lon]}
        icon={droneIcon}
      ></Marker>
    </>
  );
}

export default DroneMarker;
