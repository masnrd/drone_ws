import React, { useState, useEffect } from "react";
import markerIconPng from "leaflet/dist/images/marker-icon.png";
import L from "leaflet";
import BatteryGauge from "./BatteryGauge.js";
import { Marker, Popup, Tooltip } from "react-leaflet";

function DroneMarker(props) {
  const droneIcon = L.icon({
    iconUrl: markerIconPng,
  });
  console.log("TESTING2" + String(props.drone_id));
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
