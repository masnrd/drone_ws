import React, { useState } from "react";
// Import necessary components and icons from MUI and your project
import {
  Stepper,
  Step,
  StepLabel,
  StepContent,
  Button,
  Box,
} from "@mui/material";
import LocationSearchingIcon from "@mui/icons-material/LocationSearching";
import GpsFixedIcon from "@mui/icons-material/GpsFixed";
import DeleteIcon from "@mui/icons-material/Delete";
import WhatshotIcon from "@mui/icons-material/Whatshot";
import ListItemText from "@mui/material/ListItemText";
import ListItemIcon from "@mui/material/ListItemIcon";
import ListItem from "@mui/material/ListItem";
import IconButton from "@mui/material/IconButton";

// Assuming this component is part of your SidebarComponent
export default function ClusteringTab({
  hotspots,
  clusters,
  clustersToExplore,
  removeHotspot,
  runClustering,
  assignForSearch,
}) {
  const [activeStep, setActiveStep] = useState(0);
  const steps = [
    {
      label: "View Hotspots",
      content: (
        <>
          {/* Iterate over hotspots to display them */}
          {hotspots.map((hotspot, index) => (
            <ListItem key={index}>
              <ListItemIcon>
                <WhatshotIcon />
              </ListItemIcon>
              <ListItemText
                primary={`Position: (${hotspot[0].toFixed(
                  5
                )}, ${hotspot[1].toFixed(5)})`}
              />
              <IconButton
                aria-label="delete"
                onClick={() => removeHotspot(hotspot)}>
                <DeleteIcon />
              </IconButton>
            </ListItem>
          ))}
        </>
      ),
    },
    {
      label: "Manage Clusters",
      content: (
        <>
          {/* Iterate over clusters to display them */}
          {clusters.map((cluster, index) => (
            <ListItem key={index}>
              <ListItemIcon>
                <LocationSearchingIcon />
              </ListItemIcon>
              <ListItemText
                primary={`Position: (${cluster[0][0].toFixed(
                  5
                )}, ${cluster[0][1].toFixed(5)})`}
              />
            </ListItem>
          ))}
          <Box sx={{ display: "flex", justifyContent: "center", mt: 2 }}>
            <Button
              onClick={runClustering}
              startIcon={<GpsFixedIcon />}
              variant="outlined">
              Run Clustering
            </Button>
          </Box>
        </>
      ),
    },
    {
      label: "Assign and Search",
      content: (
        <>
          {" "}
          {clustersToExplore.map((cluster, index) => (
            <>
              <ListItem key={index}>
                <ListItemText
                  primary={`Queue Idx: ${index}`}
                  secondary={`Position: (${cluster[0][0].toFixed(
                    5
                  )}, ${cluster[0][1].toFixed(5)})`}
                />
              </ListItem>
            </>
          ))}
          <Box sx={{ display: "flex", justifyContent: "center", mt: 2 }}>
            <Button
              onClick={assignForSearch}
              startIcon={<GpsFixedIcon />}
              variant="contained">
              Assign and Search
            </Button>
          </Box>
        </>
      ),
    },
  ];

  const handleNext = () => {
    setActiveStep((prevActiveStep) => prevActiveStep + 1);
  };

  const handleBack = () => {
    setActiveStep((prevActiveStep) => prevActiveStep - 1);
  };

  return (
    <Box sx={{ mx: 2, maxWidth: 400 }}>
      <Stepper activeStep={activeStep} orientation="vertical">
        {steps.map((step, index) => (
          <Step key={index} onClick={() => setActiveStep(index)}>
            <StepLabel>
              <h3>{step.label}</h3>
            </StepLabel>
            <StepContent>{step.content}</StepContent>
          </Step>
        ))}
      </Stepper>
    </Box>
  );
}

// You would then use <ClusteringTab /> inside your SidebarComponent, passing the necessary props
