import Card from "@mui/material/Card";
import Button from "@mui/material/Button";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import LinearProgress from "@mui/material/LinearProgress";
import Box from "@mui/material/Box";
import { CardActionArea, Grid } from "@mui/material";
import Avatar from "@mui/material/Avatar";
import { useCallback } from "react";
import drone_iso from "../assets/drone_iso2.png";
import FlightLandIcon from "@mui/icons-material/FlightLand";
import SignalWifiBadIcon from "@mui/icons-material/SignalWifiBad";

const DroneStatusCard = ({ droneData, map }) => {
  function RequestLanding() {
    try {
      fetch(
        "http://127.0.0.1:5000/api/action/land?drone_id=" + droneData.drone_id
      );
    } catch (error) {
      console.error("Error:", error);
    }
  }

  function RequestDisconnect() {
    try {
      fetch(
        "http://127.0.0.1:5000/api/action/disconnect?drone_id=" +
          droneData.drone_id
      );
    } catch (error) {
      console.error("Error:", error);
    }
  }

  const center = [droneData.position.lat, droneData.position.lon];
  const zoom = 18;

  const setViewToDrone = useCallback(() => {
    map.setView(center, zoom);
  });

  let lat = droneData.position.lat;
  let lon = droneData.position.lon;
  if (lat == null || lon == null) {
    lat = "(Unknown)";
    lon = "(Unknown)";
  } else {
    lat = lat.toFixed(5);
    lon = lon.toFixed(5);
  }

  return (
    <Card sx={{ minWidth: 275, margin: 2 }}>
      <CardContent>
        <CardActionArea onClick={setViewToDrone}>
          <Grid container spacing={1}>
            <Grid item xs={3.5}>
              <Avatar
                alt="Drone"
                src={drone_iso}
                sx={{ width: 80, height: 80 }}
              />
            </Grid>
            <Grid item xs={8}>
              <Typography
                sx={{ fontSize: 14 }}
                color="text.secondary"
                gutterBottom>
                Drone ID: {droneData.drone_id}
              </Typography>
              <Typography component="div">
                Position: ({lat}, {lon})
              </Typography>
              <Typography sx={{ mb: 1.5 }} color="text.secondary">
                Mode: {droneData.mode}
              </Typography>
            </Grid>
          </Grid>
          <Box sx={{ display: "flex", alignItems: "center" }}>
            <Box sx={{ width: "100%", mr: 1 }}>
              <LinearProgress
                variant="determinate"
                value={droneData.battery_percentage}
              />
            </Box>
            <Box sx={{ minWidth: 35 }}>
              <Typography variant="body2" color="text.secondary">{`${Math.round(
                droneData.battery_percentage
              )}%`}</Typography>
            </Box>
          </Box>
          <Grid container spacing={2} sx={{ mt: 0 }}>
            <Grid item xs={6}>
              <Typography variant="body2">
                Estimated RTT: {droneData.estimated_rtt}
              </Typography>
            </Grid>
            <Grid item xs={6}>
              <Typography variant="body2">
                Last Command: {droneData.last_command || "N/A"}
              </Typography>
            </Grid>
          </Grid>
        </CardActionArea>

        <Grid container spacing={0} sx={{ mt: 2 }}>
          <Grid item xs={6} m={0}>
            <Button color="secondary" onClick={RequestLanding}>
              <FlightLandIcon /> Land
            </Button>
          </Grid>
          <Grid item xs={6} p={0}>
            <Button color="error" onClick={RequestDisconnect}>
              <SignalWifiBadIcon />
              Disconnect
            </Button>
          </Grid>
        </Grid>
      </CardContent>
    </Card>
  );
};

export default DroneStatusCard;