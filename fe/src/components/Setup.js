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
  const center = [1.3399775009363866, 103.96258672159254];

  const [markers, setMarkers] = useState([]);
  const [numberOfDrones, setNumberOfDrones] = useState(0);

  const HotspotIcon = () => (
    <SvgIcon style={{fontSize: '30px'}} viewBox="0 0 24 24">
      <svg class="w-6 h-6 text-gray-800 dark:text-white" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"
           fill="red" viewBox="0 0 24 24">
        <path
          d="M8.6 3.2a1 1 0 0 0-1.6 1 3.5 3.5 0 0 1-.8 3.6c-.6.8-4 5.6-1 10.7A7.7 7.7 0 0 0 12 22a8 8 0 0 0 7-3.8 7.8 7.8 0 0 0 .6-6.5 8.7 8.7 0 0 0-2.6-4 1 1 0 0 0-1.6.7 10 10 0 0 1-.8 3.4 9.9 9.9 0 0 0-2.2-5.5A14.4 14.4 0 0 0 9 3.5l-.4-.3Z"/>
      </svg>

    </SvgIcon>
  )
  const createIcon = () => {
    const iconHtml = ReactDOMServer.renderToString(<HotspotIcon/>);
    return L.divIcon({
      html: iconHtml,
      className: 'custom-leaflet-icon',
      iconSize: [30, 30],
      iconAnchor: [15, 30],
    });
  };

  const addMarker = (latlng) => {
    if (markers.length >= 20) {
      setMarkers(markers.slice(1));
    }
    setMarkers(prevMarkers => [...prevMarkers, {position: latlng, icon: createIcon()}]);
  };

  const removeMarker = (index) => {
    setMarkers(markers.filter((_, markerIndex) => markerIndex !== index));
  };

  const handleDronesInputChange = (event) => {
    const value = Math.max(1, Math.min(5, Number(event.target.value)));
    setNumberOfDrones(value);
  };

  async function runClustering() {
  const data = {
    numberOfDrones,
    markers,
  };

  try {
    const response = await fetch('https://your-backend-endpoint.com/api/data', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data),
    });

    if (response.ok) {
      console.log("Data sent successfully");
    } else {
      console.log("Error sending data");
    }
  } catch (error) {
    console.error("Error in sending data: ", error);
  }
}

  return (
    <div style={{display: 'flex', height: '100vh'}}>
      <div style={{width: '300px', borderRight: '1px solid black', padding: '10px'}}>
        <h4>Setup</h4>
        <div>
          <label>
            Number of Drones:
            <input
              type="number"
              value={numberOfDrones}
              onChange={handleDronesInputChange}
              min="1"
              max="5"
              style={{ marginLeft: '10px' }}
            />
          </label>
        </div>

        <h4>Selected Hotspots</h4>
        <ol>
          {markers.map((marker, index) => (
            <li key={index}>
              {`Lat: ${marker.position.lat.toFixed(3)}, Lon: ${marker.position.lng.toFixed(3)}`}
              <button onClick={() => removeMarker(index)} style={{marginLeft: '10px'}}>
                X
              </button>
            </li>
          ))}
        </ol>

        <button onClick={runClustering}>Run Clustering</button>
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
