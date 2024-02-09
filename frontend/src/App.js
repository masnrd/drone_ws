import { Route, Routes } from "react-router-dom";

import "./App.css";
import Pathfinding from "./pages/PathfindingPage";
import ClusteringPage from "./pages/ClusteringPage";
import ClusteringPagev2 from "./pages/ClusteringPagev2";

function App() {
  return (
    <>
      <Routes>
        <Route path="/" element={<Pathfinding />} />
        <Route path="/setup" element={<ClusteringPage />} />
        <Route path="/cluster" element={<ClusteringPagev2 />} />
      </Routes>
    </>
  )
}

export default App;
