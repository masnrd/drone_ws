import "./App.css";
import { Route, Routes } from "react-router-dom";
import Map from "./components/Map.js";

const App = () => {
  return (
    <div>
      <Routes>
        <Route path="/" element={<Map />} />
      </Routes>
    </div>
  );
};

export default App;
