import React from "react";
import L from "leaflet";
import ReactDOMServer from "react-dom/server";
import WhatshotIcon from "@mui/icons-material/Whatshot";
import AccessibilityIcon from "@mui/icons-material/Accessibility";
import LocationSearchingIcon from "@mui/icons-material/LocationSearching";
import DroneIconUrl from "../assets/drone2.svg";

const createDroneIcon = (number) => {
  const iconHtml = `
  <div style="position: relative; width: 100px; height: 100px; display: flex; justify-content: center; align-items: center;">
  <!-- Arrow icon (could be replaced with an SVG for more detail) with a blue background -->
  <div style="position: absolute; font-size: 24px; color: #000; background-color: light blue; width: 30px; height: 30px; display: flex; justify-content: center; align-items: center; border-radius: 50%; z-index: 10;">
  <img src="${DroneIconUrl}" style="width: 100%; height: 100%; opacity: 1;" />
  </div>
  <!-- Confounding circles with blue fill and decreasing opacity -->
  <div style="position: absolute; width: 60px; height: 60px; border-radius: 50%; background-color: rgba(100, 100, 200, 0.3);"></div>
  <div style="position: absolute; width: 30px; height: 30px; border-radius: 50%; background-color: rgba(100, 100, 200, 0.1);"></div>
  <div style="position: absolute; width: 10px; height: 10px; border-radius: 50%; background-color: rgba(100, 100, 200, 0.05);"></div>
</div>

  `;

  return L.divIcon({
    html: iconHtml,
    iconSize: [50, 50],
    iconAnchor: [25, 25],
    className: "",
  });
};

const createHotSpotIcon = () => {
  const iconHtml = ReactDOMServer.renderToString(
    <div
      style={{
        backgroundColor: "orange",
        borderRadius: "50%",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        width: "30px",
        height: "30px",
      }}>
      <WhatshotIcon style={{ color: "white", fontSize: "24px" }} />
    </div>
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
    <div
      style={{
        backgroundColor: "maroon",
        borderRadius: "50%",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        width: "30px",
        height: "30px",
      }}>
      <AccessibilityIcon style={{ color: "white", sx: 200 }} />
    </div>
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
    <div
      style={{
        backgroundColor: "#ffd214",
        borderRadius: "50%",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        width: "40px",
        height: "40px",
      }}>
      <LocationSearchingIcon style={{ fontSize: "30px", color: "white" }} />
    </div>
  );

  const iconHtml = `
    <div style="position: relative; width: 40px; height: 40px;">
      ${iconSvgHtml}
      <div style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; display: flex; justify-content: center; align-items: center; font-size: 15px; color: white;">
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
