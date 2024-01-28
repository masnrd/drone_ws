import React, { useState, useEffect } from "react";
import markerIconPng from "leaflet/dist/images/marker-icon.png";
import L from "leaflet";
import { Marker, Popup, Tooltip } from "react-leaflet";
import BatteryGauge from "./BatteryGauge.js";

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
      >
        <Popup class="bg-transparent">
          <BatteryGauge
            drone_id={props.drone_id}
            mode={props.mode}
            percentage={props.battery_percentage}
          />
        </Popup>
      </Marker>
    </>
  );
}

export default DroneMarker;
