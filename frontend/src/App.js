import { Route, Routes } from "react-router-dom";

import "./App.css";
import Pathfinding from "./pages/PathfindingPage";

function App() {
  return (
    <>
      <Routes>
        <Route path="/" element={<Pathfinding />} />
      </Routes>
    </>
  )
}

export default App;
