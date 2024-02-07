import React, { useState, useEffect} from "react";
import {Circle, MapContainer, Marker, Popup, TileLayer, useMapEvents} from 'react-leaflet';
import "leaflet/dist/leaflet.css";
import "../components/Map.css";
import L from 'leaflet';
import ReactDOMServer from 'react-dom/server';
import Tab from "../components/Tab.js";
import Sidebar from "../components/Sidebar.js";

import Typography from '@mui/material/Typography';
import Button from '@mui/material/Button';
import Box from '@mui/material/Box';
import IconButton from '@mui/material/IconButton';
import Divider from '@mui/material/Divider';

import SvgIcon from '@mui/material/SvgIcon';
import FormatListBulletedIcon from "@mui/icons-material/FormatListBulleted";
import KeyboardArrowLeftIcon from "@mui/icons-material/KeyboardArrowLeft";
import DeleteIcon from '@mui/icons-material/Delete';
import AddIcon from '@mui/icons-material/Add';
import AddLocationIcon from '@mui/icons-material/AddLocation';
import GpsFixedIcon from '@mui/icons-material/GpsFixed';
import WhatshotIcon from '@mui/icons-material/Whatshot';
import PlaceIcon from '@mui/icons-material/Place';


export default function ClusteringPagev2() {
    const start_position = [1.3430293739520736, 103.9591294705276];
    const [drones, setDrones]= useState([])
    const [hotspots, setHotspots] = useState([]);
    const [clusters, setClusters] = useState([]);
    const [clusterRadii, setClusterRadii] = useState([]);
    const [openTab, setOpenTab] = useState("home");
    const [map, setMap]= useState(null);

    const url = "http://127.0.0.1:5000/api/info";
    useEffect(() => {
      fetch(url, { mode: "cors" })
        .then((response) => response.json())
        .then((data) => {
          const hotspotData = data["hotspots"];
          const parsedHotspots = Object.values(hotspotData);
    
          // Update each hotspot object to include an icon property
          const hotspotsWithIcons = parsedHotspots.map(hotspot => ({
            ...hotspot,
            icon: createHotSpotIcon(),
          }));    
        });
        }, []);
  
    const onClose = () => {
      setOpenTab(false);
    };
  
    const onOpen = (id) => {
      setOpenTab(id);
    };

    async function runClustering() {
        try {
          const params = new URLSearchParams();
          params.append('hotspots_position', JSON.stringify(hotspots));
    
          const response = await fetch('http://127.0.0.1:5000/api/setup/run_clustering', {
            method: 'POST',
            body: params,
          });
          if (response.ok) {
            const responseData = await response.json();
            const clusterData = Object.values(responseData).map(cluster => ({
              position: {
                lat: cluster[0][0],
                lng: cluster[0][1]
              },
              icon: createClusterIcon()
            }));
            const radiiData = Object.values(responseData).map(cluster => cluster[1]);
    
            setClusters(clusterData);
            setClusterRadii(radiiData);
            
          } else {
            console.error('Error:', response.status, response.statusText);
          }
    
        } catch (error) {
          console.error('Error:', error);
        }
      }

    async function confirmClustering(){
  
      try {
        const params = new URLSearchParams();
        params.append('clusters', JSON.stringify(clusters));
  
        const response = await fetch('http://127.0.0.1:5000/api/setup/confirm_clustering', {
          method: 'POST',
          body: params,
        });
        if (response.ok) {
        } else {
          console.error('Error:', response.status, response.statusText);
        }
  
      } catch (error) {
        console.error('Error:', error);
      }
    }

    

    async function assignForSearch(){
  
      try {  
        const response = await fetch('http://127.0.0.1:5000/api/setup/start_operation', {
          method: 'GET',
        });
        if (response.ok) {
        } else {
          console.error('Error:', response.status, response.statusText);
        }
  
      } catch (error) {
        console.error('Error:', error);
      }
    }


  
    function getNewPosition(lat, lon, distanceInMeters) {
      const EarthRadius = 6378137;
      const dLat = distanceInMeters / EarthRadius;
      const dLatDegrees = dLat * (180 / Math.PI);
  
      return {
        lat: lat + dLatDegrees,
        lon: lon
      };
    }
      const addHotspot = (latlng) => {
          if (hotspots.length >= 20) {
              setHotspots(hotspots.slice(1));
          }
          setHotspots(prevHotspots => [...prevHotspots, {position: latlng, icon: createHotSpotIcon()}]);
      };
      const removeHotspot = (index) => {
          setHotspots(hotspots.filter((_, hotspotIndex) => hotspotIndex !== index));
      };

      const createHotSpotIcon = () => {
        const iconHtml = ReactDOMServer.renderToString(<WhatshotIcon style={{ color: 'red', sx:200}} />);
        return L.divIcon({
            html: iconHtml,
            className: 'custom-leaflet-icon', // Adjust as needed, ensure this class does minimal or no styling to avoid conflicts
            iconSize: L.point(30, 30), // Adjust size as needed
            iconAnchor: L.point(15, 30), // Adjust anchor as needed, consider the icon size
        });
    };
    
    const createClusterIcon = () => {
        const iconHtml = ReactDOMServer.renderToString(<PlaceIcon style={ {color:'blue', sx:200}}/>);
        return L.divIcon({
            html: iconHtml,
            className: 'custom-leaflet-cluster-icon', // Adjust as needed, ensure this class does minimal or no styling to avoid conflicts
            iconSize: L.point(30, 30), // Adjust size as needed
            iconAnchor: L.point(15, 30), // Adjust anchor as needed, consider the icon size
        });
    };
    
    const createNumberIcon = (number) => {
      const iconHtml = `<div style="background-color: white; color: black; border-radius: 50%; width: 30px; height: 30px; display: flex; justify-content: center; align-items: center; font-size: 12px; border: 1px solid black;">${number}</div>`;
      return L.divIcon({
          html: iconHtml,
          className: 'my-custom-icon', // Ensure this class does minimal or no styling to avoid conflicts
          iconSize: [30, 30], // Adjust size as needed
          iconAnchor: [15, 30], // Adjust anchor as needed, consider the icon size
      });
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
          <Box sx={{ padding: 2 }}>
          <h2>Hotspots</h2>
      {hotspots.map((hotspot, index) => (
        <React.Fragment key={index}>
          <Box sx={{ marginBottom: 2, display: 'flex', alignItems: 'center', gap: 2 }}>
            <AddIcon color="primary" />
            <Typography>Latitude: {hotspot.position.lat.toFixed(3)} Longitude: {hotspot.position.lng.toFixed(3)}</Typography>
            <IconButton aria-label="delete" onClick={() => removeHotspot(index)}>
              <DeleteIcon />
            </IconButton>
          </Box>
          {index < hotspots.length - 1 && <Divider />}
        </React.Fragment>
      ))}
    </Box>
    <h2>Clusters</h2>
    {clusters.map((cluster, index) => (
        <React.Fragment key={index}>
          <Box sx={{ marginBottom: 2, display: 'flex', alignItems: 'center', gap: 2 }}>
            <AddLocationIcon color="primary" />
            <Typography>Lat: {cluster.position.lat.toFixed(3)} Lon: {cluster.position.lng.toFixed(3)}</Typography>
            {clusterRadii[index] ? <Typography>Radius {clusterRadii[index].toFixed(2)}m</Typography> : <div/>}
            {/* <IconButton aria-label="delete" onClick={() => removeHotspot(index)}>
              <DeleteIcon />
            </IconButton> */}
          </Box>
          {index < hotspots.length - 1 && <Divider />}
        </React.Fragment>
      ))}
    <Box sx={{ display: 'flex', justifyContent: 'center', marginTop: 2 }}>
        <Button variant="contained" color="primary" onClick={runClustering} startIcon={<GpsFixedIcon />}>
          Run Clustering
        </Button>
        <Button variant="contained" color="primary" onClick={confirmClustering} startIcon={<GpsFixedIcon />}>
          Confirm Clusters
        </Button>
        <Button variant="contained" color="primary" onClick={assignForSearch} startIcon={<GpsFixedIcon />}>
          Assign and Search
        </Button>
      </Box>
      </Tab>
        </Sidebar>
        <MapContainer
          center={start_position}
          zoom={18}
          minZoom={16}
          maxZoom={30}
          scrollWheelZoom={true}
          ref={setMap}
        >       
        <TileLayer
            attribution='&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          />
          <AddHotspotOnClick onNewPoint={addHotspot}/>
          {hotspots.map((hotspot, index) => (
            <Marker
              key={index}
              position={hotspot.position}
              icon={hotspot.icon}>
              <Popup>
                {`Latitude: ${hotspot.position.lat}, Longitude: ${hotspot.position.lng}`}
              </Popup>
            </Marker>
          ))}
  
          {clusters.map((cluster, index) => (
            cluster.position && (
              <>
                <Marker
                  key={`marker-${index}`}
                  position={cluster.position}
                  icon={cluster.icon}>
                  <Popup>
                    {`Latitude: ${cluster.position.lat}, Longitude: ${cluster.position.lng}`}
                  </Popup>
                </Marker>
                <Circle
                  key={`circle-${index}`}
                  center={cluster.position}
                  radius={clusterRadii[index]}
                  color="blue"
                  fillColor="blue"
                  fillOpacity={0.2}
                />
                <Marker
                  key={`number-${index}`}
                  position={getNewPosition(cluster.position.lat, cluster.position.lng, 20)}
                  icon={createNumberIcon(index + 1)}
                  zIndexOffset={1000}
                />
              </>
            )
          ))}    
        </MapContainer>
      </>
    );
  }
  
  function AddHotspotOnClick({onNewPoint}) {
      useMapEvents({
        click(e) {
          onNewPoint(e.latlng);
        },
      });
    
      return null;
    }