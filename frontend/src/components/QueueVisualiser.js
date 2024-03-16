import React from "react";
import List from "@mui/material/List";
import ListItem from "@mui/material/ListItem";
import ListItemText from "@mui/material/ListItemText";
import Paper from "@mui/material/Paper";

const QueueVisualization = ({ clusters }) => {
  return (
    <Paper elevation={3} sx={{ maxWidth: 360, margin: "auto" }}>
      <List>
        {clusters.map((cluster, index) => (
          <>
            <ListItem key={index} divider={index !== clusters.length - 1}>
              <ListItemText
                primary={`Queue Index: ${index}`}
                secondary={`Position: (${cluster[0][0].toFixed(
                  5)}, ${cluster[0][1].toFixed(5)})`}
              />
            </ListItem>
          </>
        ))}
      </List>
    </Paper>
  );
};

export default QueueVisualization;
