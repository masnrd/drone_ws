# Altitude of the drone, in METRES.
export ALTITUDE_M=7.0

# Distance from a LatLon for the drone to consider it "reached", in METRES.
export THRESH_REACHED_M=1

# Distance from the ground for the drone to consider itself "landed", in METRES.
export THRESH_LANDED_M=0.5

# Horizontal speed of the drone, in METRES PER SECOND.
export SPEED_HORIZONTAL_MPS=0.334

# Vertical speed of the drone, in METRES PER SECOND.
export SPEED_VERTICAL_MPS=0.2

# How long the drone spends in the air without receiving instructions from mission control.
# Note that in this state, the drone is CONNECTED to MC, the timeout for disconnection is not this constant.
# In SECONDS.
export IDLE_AIR_TIMEOUT_S=300

# How long the drone spends in the air WITHOUT BEING CONNECTED TO MISSION CONTROL.
# In SECONDS.
export CONN_AIR_TIMEOUT_S=15

# Seconds until we consider a loss of connection to the flight controller.
export FC_LOSS_TIMEOUT_S=1

# Seconds until we consider a loss of connection to the mission control.
export MC_LOSS_TIMEOUT_S=3

# Seconds to try receiving messages from the flight controller upon initialisation.
# This is deliberately longer, to account for startup delay
export FC_INITIAL_TIMEOUT_S=60