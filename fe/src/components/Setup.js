import React, {useState} from 'react';
import {Circle, MapContainer, Marker, Popup, TileLayer, useMapEvents} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import SvgIcon from '@mui/material/SvgIcon';
import L from 'leaflet';
import ReactDOMServer from 'react-dom/server';

function AddHotspotOnClick({onNewPoint}) {
  useMapEvents({
    click(e) {
      onNewPoint(e.latlng);
    },
  });

  return null;
}

function Setup() {
  const center = [1.3399775009363866, 103.96258672159254];

  const [hotspots, setHotspots] = useState([]);
  const [numberOfDrones, setNumberOfDrones] = useState(1);

  const [clusters, setClusters] = useState([]);
  const [clusterRadii, setClusterRadii] = useState([]);


  const HotspotIcon = () => (
    <SvgIcon style={{fontSize: '30px'}} viewBox="0 0 24 24">
      <svg class="w-6 h-6 text-gray-800 dark:text-white" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"
           fill="red" viewBox="0 0 24 24">
        <path
          d="M8.6 3.2a1 1 0 0 0-1.6 1 3.5 3.5 0 0 1-.8 3.6c-.6.8-4 5.6-1 10.7A7.7 7.7 0 0 0 12 22a8 8 0 0 0 7-3.8 7.8 7.8 0 0 0 .6-6.5 8.7 8.7 0 0 0-2.6-4 1 1 0 0 0-1.6.7 10 10 0 0 1-.8 3.4 9.9 9.9 0 0 0-2.2-5.5A14.4 14.4 0 0 0 9 3.5l-.4-.3Z"/>
      </svg>
    </SvgIcon>
  )
  const ClusterIcon = () => (
    <SvgIcon style={{fontSize: '30px'}} viewBox="0 0 24 24">
      <svg className="w-6 h-6 text-gray-800 dark:text-white" aria-hidden="true" xmlns="http://www.w3.org/2000/svg"
           fill="blue" viewBox="0 0 24 24">
        <path stroke="blue" stroke-linecap="round" stroke-linejoin="round" stroke-width="2"
              d="M12 13a3 3 0 1 0 0-6 3 3 0 0 0 0 6Z"/>
        <path stroke="blue" stroke-linecap="round" stroke-linejoin="round" stroke-width="2"
              d="M17.8 14h0a7 7 0 1 0-11.5 0h0l.1.3.3.3L12 21l5.1-6.2.6-.7.1-.2Z"/>
      </svg>
    </SvgIcon>
  );

  const createClusterIcon = () => {
    const iconHtml = ReactDOMServer.renderToString(<ClusterIcon/>);
    return L.divIcon({
      html: iconHtml,
      className: 'custom-leaflet-cluster-icon',
      iconSize: [60, 60],
      iconAnchor: [30, 60],
    });
  };
  const createHotSpotIcon = () => {
    const iconHtml = ReactDOMServer.renderToString(<HotspotIcon/>);
    return L.divIcon({
      html: iconHtml,
      className: 'custom-leaflet-icon',
      iconSize: [30, 30],
      iconAnchor: [15, 30],
    });
  };

  function createNumberIcon(number) {
    return L.divIcon({
      className: 'my-custom-icon',
      html: `<div style="background-color: white; border-radius: 50%; width: 25px; height: 25px; display: flex; justify-content: center; align-items: center; font-size: 12px; border: 1px solid black;">${number}</div>`,
      iconSize: [25, 25],
      iconAnchor: [12, 12]
    });
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

  const handleDronesInputChange = (event) => {
    const value = Math.max(1, Math.min(5, Number(event.target.value)));
    setNumberOfDrones(value);
  };

  async function runClustering() {
    const hotspots_position = hotspots.map((hotspot) => ({
      lat: hotspot.position.lat,
      lng: hotspot.position.lng,
    }));

    try {
      const params = new URLSearchParams();
      params.append('numberOfDrones', numberOfDrones);
      params.append('hotspots_position', JSON.stringify(hotspots_position));

      const response = await fetch('http://localhost:5000/api/setup/run_clustering', {
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
        console.log(responseData)
        console.log(clusterData)
        console.log(radiiData)
      } else {
        console.error('Error:', response.status, response.statusText);
      }

    } catch (error) {
      console.error('Error:', error);
    }
  }

  return (
    <div style={{display: 'flex', height: '100vh'}}>
      <div
        style={{width: '300px', borderRight: '1px solid black', padding: '10px', height: '100vh', overflowY: 'auto'}}>
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
              style={{marginLeft: '10px'}}
            />
          </label>
        </div>

        <h4>Selected Hotspots</h4>
        <ol>
          {hotspots.map((hotspot, index) => (
            <li key={index}>
              {`Lat: ${hotspot.position.lat.toFixed(3)}, Lon: ${hotspot.position.lng.toFixed(3)}`}
              <button onClick={() => removeHotspot(index)} style={{marginLeft: '10px'}}>
                X
              </button>
            </li>
          ))}
        </ol>

        <button onClick={runClustering}>Run Clustering</button>

        <h4>Cluster Results</h4>
        <ol>
          {clusters.map((cluster, index) => (
            <li key={index}>
              {`Coor: (${cluster.position.lat.toFixed(3)}, ${cluster.position.lng.toFixed(3)})`}
              <br/>
              {`Radius: ${clusterRadii[index].toFixed(2)}m`}
            </li>
          ))}
        </ol>
      </div>
      <MapContainer center={center} zoom={13} style={{flex: 1}}>
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
                icon={createNumberIcon(index)}
                zIndexOffset={1000}
              />
            </>
          )
        ))}
      </MapContainer>
    </div>
  );
}

export default Setup;
