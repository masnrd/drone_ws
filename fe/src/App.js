import { Route, Routes } from "react-router-dom";

import "./App.css";
import Map from "./components/Map.js";
import Setup from "./components/Setup.js";

const App = () => {
  return (
    <div>
      <Routes>
        <Route path="/" element={<Map />} />
        <Route path="/setup" element={<Setup />} />
      </Routes>
    </div>
  );
};

export default App;
