import { Route, Routes } from "react-router-dom";

import "./App.css";
import Pathfinding from "./pages/PathfindingPage";
import Setup from "./components/Setup";

function App() {
  return (
    <>
      <Routes>
        <Route path="/" element={<Pathfinding />} />
        <Route path="/setup" element={<Setup />} />
      </Routes>
    </>
  )
}

export default App;
