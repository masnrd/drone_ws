import React, { useState } from "react";
import Tab from "./Tab.js";
import Sidebar from "./Sidebar.js";
import DroneStatusCard from "./DroneStatusCard.js";

import SensorOccupiedIcon from "@mui/icons-material/SensorOccupied";
import KeyboardArrowLeftIcon from "@mui/icons-material/KeyboardArrowLeft";
import LocationSearchingIcon from "@mui/icons-material/LocationSearching";
import AccessibilityIcon from "@mui/icons-material/Accessibility";
import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import Divider from "@mui/material/Divider";
import FakeDetection from "./FakeDetection";
import ListItem from "@mui/material/ListItem";
import ClusteringTab from "./ClusteringTab.js";

export default function SidebarComponent({
  map,
  drones,
  hotspots,
  clusters,
  clustersToExplore,
  detectedEntities,
}) {
  const [openTab, setOpenTab] = useState("home");

  const onClose = () => {
    setOpenTab(false);
  };

  const onOpen = (id) => {
    setOpenTab(id);
  };
  const removeHotspot = (latlng) => {
    const url = "http://127.0.0.1:5000/hotspot/delete";
    const params = new URLSearchParams();
    params.append("hotspot_position", JSON.stringify({ latlng }));
    fetch(url, { method: "POST", body: params });
  };

  function runClustering() {
    try {
      fetch("http://127.0.0.1:5000/api/setup/run_clustering");
    } catch (error) {
      console.error("Error:", error);
    }
  }

  const assignForSearch = () => {
    try {
      fetch("http://127.0.0.1:5000/api/setup/start_operation");
    } catch (error) {
      console.error("Error:", error);
    }
  };

  return (
    <>
      <Sidebar
        map={map}
        position="left"
        collapsed={!openTab}
        selected={openTab}
        closeIcon={<KeyboardArrowLeftIcon />}
        onClose={onClose}
        onOpen={onOpen}
        panMapOnChange
        rehomeControls>
        <Tab id="home" header="Drones" icon={<SensorOccupiedIcon />} active>
          {drones.map((drone) => (
            <DroneStatusCard key={drone.drone_id} droneData={drone} map={map} />
          ))}
        </Tab>
        <Tab
          id="clustering"
          header="Clustering"
          icon={<LocationSearchingIcon />}
          active>
          <ClusteringTab
            hotspots={hotspots}
            clusters={clusters}
            removeHotspot={removeHotspot}
            runClustering={runClustering}
            assignForSearch={assignForSearch}
            clustersToExplore={clustersToExplore}
          />
        </Tab>
        <Tab
          id="detected"
          header="Detection Status"
          icon={<AccessibilityIcon />}
          active>
          {detectedEntities
            .filter((entity) =>
              FakeDetection.shouldDisplayEntity(entity, clusters)
            )
            .map((entity, index) => (
              <ListItem key={index}>
                <Box
                  sx={{
                    marginBottom: 2,
                    display: "flex",
                    alignItems: "center",
                    gap: 2,
                  }}>
                  <AccessibilityIcon style={{ color: "bright orange" }} />
                  <Typography>
                    Detection {index + 1}: ({entity.coordinates.lat.toFixed(5)},{" "}
                    {entity.coordinates.lon.toFixed(5)})
                  </Typography>
                </Box>
                {index < detectedEntities.length - 1 && (
                  <Divider variant="middle" />
                )}
              </ListItem>
            ))}
        </Tab>
      </Sidebar>
    </>
  );
}