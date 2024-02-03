import { Route, Routes } from "react-router-dom";

import "./App.css";
import Pathfinding from "./pages/PathfindingPage";
import ClusteringPage from "./pages/ClusteringPage";

function App() {
  return (
    <>
      <Routes>
        <Route path="/" element={<Pathfinding />} />
        <Route path="/setup" element={<ClusteringPage />} />
      </Routes>
    </>
  )
}

export default App;
