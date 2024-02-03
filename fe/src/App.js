import "./App.css";
import { Route, Routes } from "react-router-dom";
import Pathfinding from "./pages/PathfindingPage";
import { useState } from "react";
import { Path } from "leaflet";

const App = () => {
  const [map, setMap] = useState(null);
  return (
    <div>
      <Routes>
        <Route path="/" element={<Pathfinding />} />
      </Routes>
    </div>
  );
};

export default App;
