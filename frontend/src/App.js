import { Route, Routes } from "react-router-dom";

import "./App.css";
import Pathfinding from "./pages/PathfindingPage";
import { createTheme, ThemeProvider } from "@mui/material/styles";
import { orange, blue } from "@mui/material/colors";

function App() {
  const theme = createTheme({
    palette: {
      primary: {
        main: blue[800],
      },
      secondary: {
        main: orange[700],
      },
    },
  });

  return (
    <>
      <ThemeProvider theme={theme}>
        <Routes>
          <Route path="/" element={<Pathfinding />} />
        </Routes>
      </ThemeProvider>
    </>
  );
}

export default App;