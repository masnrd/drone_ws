import Card from "@mui/material/Card";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import LinearProgress from "@mui/material/LinearProgress";
import Box from "@mui/material/Box";
import { CardActionArea, Grid } from "@mui/material";
import { useCallback } from "react";

const DroneStatusCard = ({ droneData, map }) => {
  console.log("map", map);
  const center = [droneData.lat, droneData.lon];
  const zoom = 18;

  const setViewToDrone = useCallback(() => {
    map.setView(center, zoom);
  });

  return (
    <Card sx={{ minWidth: 275, marginBottom: 2 }}>
      <CardActionArea onClick={setViewToDrone}>
        <CardContent>
          <Typography sx={{ fontSize: 14 }} color="text.secondary" gutterBottom>
            Drone ID: {droneData._drone_id}
          </Typography>
          <Typography variant="h5" component="div">
            Position: ({droneData.lat.toFixed(3)}, {droneData.lon.toFixed(3)})
          </Typography>
          <Typography sx={{ mb: 1.5 }} color="text.secondary">
            Mode: {droneData.mode}
          </Typography>
          <Box sx={{ display: "flex", alignItems: "center" }}>
            <Box sx={{ width: "100%", mr: 1 }}>
              <LinearProgress
                variant="determinate"
                value={droneData._battery_percentage}
              />
            </Box>
            <Box sx={{ minWidth: 35 }}>
              <Typography variant="body2" color="text.secondary">{`${Math.round(
                droneData._battery_percentage
              )}%`}</Typography>
            </Box>
          </Box>
          <Grid container spacing={2} sx={{ mt: 2 }}>
            <Grid item xs={6}>
              <Typography variant="body2">
                Estimated RTT: {droneData._estimated_rtt}
              </Typography>
            </Grid>
            <Grid item xs={6}>
              <Typography variant="body2">
                Last Command: {droneData._last_command || "N/A"}
              </Typography>
            </Grid>
          </Grid>
        </CardContent>
      </CardActionArea>
    </Card>
  );
};

export default DroneStatusCard;
