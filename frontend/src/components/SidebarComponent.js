import React, {useState} from "react";
import Tab from "./Tab.js";
import Sidebar from "./Sidebar.js";
import DroneStatusCard from "./DroneStatusCard.js";

import SensorOccupiedIcon from '@mui/icons-material/SensorOccupied';
import KeyboardArrowLeftIcon from "@mui/icons-material/KeyboardArrowLeft";
import DeleteIcon from "@mui/icons-material/Delete";
import WhatshotIcon from "@mui/icons-material/Whatshot";
import GpsFixedIcon from "@mui/icons-material/GpsFixed";
import LocationSearchingIcon from '@mui/icons-material/LocationSearching';
import AccessibilityIcon from "@mui/icons-material/Accessibility";
import Typography from "@mui/material/Typography";
import Box from "@mui/material/Box";
import IconButton from "@mui/material/IconButton";
import Divider from "@mui/material/Divider";
import Button from "@mui/material/Button";
import QueueVisualization from "./QueueVisualiser.js";
import FakeDetection from "./FakeDetection";


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
    params.append("hotspot_position", JSON.stringify({latlng}));
    fetch(url, {method: "POST", body: params});
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
        closeIcon={<KeyboardArrowLeftIcon/>}
        onClose={onClose}
        onOpen={onOpen}
        panMapOnChange
        rehomeControls>
        <Tab id="home" header="Drones" icon={<SensorOccupiedIcon/>} active>
          {drones.map((drone) => (
            <DroneStatusCard key={drone.drone_id} droneData={drone} map={map}/>
          ))}
        </Tab>
        <Tab
          id="clustering"
          header="Clustering"
          icon={<LocationSearchingIcon/>}
          active>
          <Box sx={{padding: 2}}>
            <h1>Hotspot</h1>
            {hotspots.map((hotspot, index) => (
              <React.Fragment key={index}>
                <Box
                  sx={{
                    marginBottom: 2,
                    display: "flex",
                    alignItems: "center",
                    gap: 2,
                  }}>
                  <WhatshotIcon style={{color: "red", sx: 200}}/>
                  <Typography>
                    Position: ({hotspot[0].toFixed(5)}, {hotspot[1].toFixed(5)})
                  </Typography>
                  <IconButton
                    aria-label="delete"
                    onClick={() => removeHotspot(hotspot)}>
                    <DeleteIcon/>
                  </IconButton>
                </Box>
                {index < hotspots.length - 1 && <Divider/>}
              </React.Fragment>
            ))}
            <h1>Clusters</h1>
            {clusters.map((cluster, index) => (
              <div key={`marker-${index}`}>
                <Box
                  sx={{
                    marginBottom: 2,
                    display: "flex",
                    alignItems: "center",
                    gap: 2,
                  }}>
                  <LocationSearchingIcon style={{color: "blue", sx: 200}}/>
                  <Typography>
                    Position: ({cluster[0][0].toFixed(5)}, {cluster[0][1].toFixed(5)})
                  </Typography>
                </Box>
                {index < hotspots.length - 1 && <Divider/>}
              </div>
            ))}
          </Box>
          <Box sx={{display: "flex", justifyContent: "center", marginTop: 2}}>
            <Button
              variant="outlined"
              color="primary"
              onClick={runClustering}
              startIcon={<GpsFixedIcon/>}>
              Run Clustering
            </Button>
          </Box>
          <Box sx={{display: "flex", justifyContent: "center", marginTop: 2}}>
            <Button
              variant="contained"
              color="primary"
              onClick={assignForSearch}
              startIcon={<GpsFixedIcon/>}>
              Assign and Search
            </Button>
          </Box>
          <h1>Queue</h1>
          <QueueVisualization clusters={clustersToExplore}/>
        </Tab>
        <Tab
          id="detected"
          header="Detection Status"
          icon={<AccessibilityIcon/>}
          active>
          {detectedEntities.filter(entity => FakeDetection.shouldDisplayEntity(entity, clusters)).map((entity, index) => (
            <div key={`entity-info-${index}`}>
              <Box
                sx={{
                  marginBottom: 2,
                  display: "flex",
                  alignItems: "center",
                  gap: 2,
                }}>
                <AccessibilityIcon style={{color: "bright orange"}}/>
                <Typography>
                  Detection {index+1}: ({entity.coordinates.lat.toFixed(5)}, {entity.coordinates.lon.toFixed(5)})
                </Typography>
              </Box>
              {index < detectedEntities.length - 1 && <Divider/>}
            </div>
          ))}
        </Tab>
      </Sidebar>
    </>
  );
}
