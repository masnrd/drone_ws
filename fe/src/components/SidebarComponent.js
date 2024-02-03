import React, { useState, useEffect, useRef } from "react";
import Tab from "./Tab.js";
import Sidebar from "./Sidebar.js";
import DroneStatusCard from "./DroneStatusCard.js";

import FormatListBulletedIcon from "@mui/icons-material/FormatListBulleted";
import KeyboardArrowLeftIcon from "@mui/icons-material/KeyboardArrowLeft";

function SidebarComponent({ map, drones }) {
  const [openTab, setOpenTab] = useState("home");
  const onClose = () => {
    setOpenTab(false);
  };

  const onOpen = (id) => {
    setOpenTab(id);
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
        rehomeControls
      >
        <Tab id="home" header="Drones" icon={<FormatListBulletedIcon />} active>
          {drones.map((drone) => (
            <DroneStatusCard droneData={drone} map={map} />
          ))}
        </Tab>
      </Sidebar>
    </>
  );
}

export default SidebarComponent;
