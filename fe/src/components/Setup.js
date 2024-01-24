import React, {useState} from 'react';
import {MapContainer, TileLayer, Marker, Popup, useMapEvents} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import SvgIcon from '@mui/material/SvgIcon';
import L from 'leaflet';
import ReactDOMServer from 'react-dom/server';

function AddMarkerOnClick({onNewPoint}) {
  useMapEvents({
    click(e) {
      onNewPoint(e.latlng);
    },
  });

  return null;
}

function Setup() {
  const [markers, setMarkers] = useState([]);
  const center = [51.505, -0.09]; // Set your initial map center coordinates

  const CustomIcon = () => (
    <SvgIcon style={{fontSize: '30px'}} viewBox="0 0 24 24">
      <svg class="w-6 h-6 text-gray-800 dark:text-white" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"
           fill="red" viewBox="0 0 24 24">
        <path
          d="M8.6 3.2a1 1 0 0 0-1.6 1 3.5 3.5 0 0 1-.8 3.6c-.6.8-4 5.6-1 10.7A7.7 7.7 0 0 0 12 22a8 8 0 0 0 7-3.8 7.8 7.8 0 0 0 .6-6.5 8.7 8.7 0 0 0-2.6-4 1 1 0 0 0-1.6.7 10 10 0 0 1-.8 3.4 9.9 9.9 0 0 0-2.2-5.5A14.4 14.4 0 0 0 9 3.5l-.4-.3Z"/>
      </svg>

    </SvgIcon>
  );

// Function to create a custom Leaflet icon
  const createIcon = () => {
    const iconHtml = ReactDOMServer.renderToString(<CustomIcon/>);
    return L.divIcon({
      html: iconHtml,
      className: 'custom-leaflet-icon',
      iconSize: [30, 30], // Adjust size as needed
      iconAnchor: [15, 30], // Adjust anchor point as needed
    });
  };

  const addMarker = (latlng) => {
    setMarkers([...markers, {position: latlng, icon: createIcon()}]);
  };

  return (
    <div style={{display: 'flex', height: '100vh'}}>
      <div style={{width: '300px', borderRight: '1px solid black', padding: '10px'}}>
        <h4>Clicked Points</h4>
        <ul>
          {markers.map((marker, index) => (
            <li key={index}>{`Latitude: ${marker.position.lat}, Longitude: ${marker.position.lng}`}</li>
          ))}
        </ul>
      </div>
      <MapContainer center={center} zoom={13} style={{flex: 1}}>
        <TileLayer
          attribution='&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <AddMarkerOnClick onNewPoint={addMarker}/>
        {markers.map((marker, index) => (
          <Marker
            key={index}
            position={marker.position}
            icon={marker.icon}>
            <Popup>
              {`Latitude: ${marker.position.lat}, Longitude: ${marker.position.lng}`}
            </Popup>
          </Marker>
        ))}
      </MapContainer>
    </div>
  );
}

export default Setup;
