import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import LinearProgress from "@mui/material/LinearProgress";
import Box from "@mui/material/Box";
import { CardActionArea, Grid } from "@mui/material";
import { useCallback } from "react";

const DroneStatusCard = ({ droneData, map }) => {
  const center = [droneData.position.lat, droneData.position.lon];
  const zoom = 18;

  const setViewToDrone = useCallback(() => {
    map.setView(center, zoom);
  });

  let lat = droneData.position.lat
  let lon = droneData.position.lon
  if (lat == null || lon == null) {
    lat = "(Unknown)";
    lon = "(Unknown)";
  } else {
    lat = lat.toFixed(3);
    lon = lon.toFixed(3);
  }

  return (
    <Card sx={{ minWidth: 275, marginBottom: 2 }}>
      <CardActionArea onClick={setViewToDrone}>
        <CardContent>
          <Typography sx={{ fontSize: 14 }} color="text.secondary" gutterBottom>
            Drone ID: {droneData.drone_id}
          </Typography>
          <Typography variant="h5" component="div">
            Position: ({lat}, {lon})
          </Typography>
          <Typography sx={{ mb: 1.5 }} color="text.secondary">
            Mode: {droneData.mode}
          </Typography>
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
          <Grid container spacing={2} sx={{ mt: 2 }}>
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
        </CardContent>
      </CardActionArea>
    </Card>
  );
};

export default DroneStatusCard;
