import { Route, Routes } from "react-router-dom";

import "./App.css";
import Map from "./components/Map.js";
import Config from "./components/Config.js";

const App = () => {
  return (
    <div>
      <Routes>
        <Route path="/" element={<Map />} />
        <Route path="/config" element={<Config />} />
      </Routes>
    </div>
  );
};

export default App;
