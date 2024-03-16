import React from "react";
import L from "leaflet";
import ReactDOMServer from "react-dom/server";
import WhatshotIcon from "@mui/icons-material/Whatshot";
import AccessibilityIcon from "@mui/icons-material/Accessibility";
import LocationSearchingIcon from '@mui/icons-material/LocationSearching';
import DroneIconUrl from "../assets/drone.svg"

const createDroneIcon = (number) => {
  const iconHtml = `
    <div style="position: relative; width: 50px; height: 50px;">
      <img src="${DroneIconUrl}" style="width: 100%; height: 100%; opacity: 0.7;" />
      <div style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; display: flex; justify-content: center; align-items: center; font-size: 15px; color: whitesmoke;">
        ${number}
      </div>
    </div>
  `;

  return L.divIcon({
    html: iconHtml,
    iconSize: [50, 50],
    iconAnchor: [25, 25],
    className: ''
  });
};

const createHotSpotIcon = () => {
  const iconHtml = ReactDOMServer.renderToString(
    <WhatshotIcon style={{color: "red", sx: 200}}/>
  );
  return L.divIcon({
    html: iconHtml,
    className: "custom-leaflet-icon",
    iconSize: L.point(30, 30),
    iconAnchor: L.point(15, 15),
  });
};

const createDetectionIcon = () => {
  const iconHtml = ReactDOMServer.renderToString(
    <AccessibilityIcon style={{color: "bright orange", sx: 200}}/>
  );
  return L.divIcon({
    html: iconHtml,
    className: "custom-leaflet-icon",
    iconSize: L.point(30, 30),
    iconAnchor: L.point(15, 15),
  });
};

const createClusterIcon = (number) => {
  const iconSvgHtml = ReactDOMServer.renderToString(
    <LocationSearchingIcon style={{fontSize: '30px', color: 'blue'}}/>
  );

  const iconHtml = `
    <div style="position: relative; width: 30px; height: 30px;">
      ${iconSvgHtml}
      <div style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; display: flex; justify-content: center; align-items: center; font-size: 15px; color: blue;">
        ${number}
      </div>
    </div>
  `;

  return L.divIcon({
    html: iconHtml,
    className: "custom-leaflet-cluster-icon",
    iconSize: [30, 30],
    iconAnchor: [15, 15],
  });
};

export default {
  createDroneIcon,
  createHotSpotIcon,
  createDetectionIcon,
  createClusterIcon,
};
