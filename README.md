# ground-control-ui

## Frontend Features

### 1. Mission Control Main Page

#### Frontend Features

1. View map
   1. With h3 hexagon tiling representation overlay
   2. Zoom into a specific long lat
2. Individual drone slide in panel
   1. Drone info
   2. Battery
   3. Status
   4. Location (Longitude , Latitude)
   5. Stage of Search and Rescue
   6. Next action? (Since drones are independent, possible to also send projected new actions like a log of subsequent actions?)
3. Drone Manual Control
   1. Search Cluster - must have an associated cluster to search (i.e. given a cluster and its associated hotspots, search the hexagons - exhaustively?)
   2. Return to Base
   3. Move to location? (Longitude, Latitude)
   4. Manual Control via QGC
4. Drone Summary Page
   1. List of all drones and info in individual drone panel (Refer 2.)
5. Clustering Setup Page
   1. Import clusters using csv
   2. Visualise clusters
   3. Approve/Disappove
   4. Manual cluster assignment (TODO: Criteria to check for valid clusters - maximum span of cluster)
6. (TBC) Drone assignment UI feature to manually/automatically assign drones to specific clusters
   1. Queue to visualise available drones
   2. Queue to visualise pending clusters
   3. (TBC) Progress search indicator for each cluster

#### Backend Features

1. Run clustering algorithm given points
   1. Fail safe checks for hotpost input
2. Handle back to base etc.
3. Drone assignment algorithm
4. Handle manual drone assignment
   1. Fail safe checks for task assignment e.g. battery level, distance from homebase

#### ROS 2 Node API Features

1. All drone info - `api/info`
2. Move to a position, then idle - `/api/action/moveto?drone_id={DRONE ID}&lat={LATITUDE}&lon={LONGITUDE}`
3. Move to a position, then begin search at that coordinate - `/api/action/search?drone_id={DRONE ID}&lat={LATITUDE}&lon={LONGITUDE}`
   (TODO)
4. Downgrade (to QGC) - `tbc`
