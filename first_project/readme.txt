ROS Project #1 - 2025
Riva, Staffaroni Biagetti, Meshcheriakov

Node 1 (odometer):
For the odometer node exact integration is used. We used two thresholds, one for speed and one for steering angle, so that when these quantities
are to small the integration is skipped for that specific dt. We saw that doing that the odometry was closer to the gps position, probably because 
noise from sensor was partially filtered.

Node 2 (gps odometer):
For the GPS Node we did this passages to obtain valid data.First we recieved longitude,altitude and latitude.
Secondly,transformed them to ECEF cooordiantes and after in ENU coordinates.
As base point we decide to take first published data from gps.To calculate orientation ,we subtract from recent position the last position of the car.

Node 3 (sector times):
Time for sector 1 was measured from the beginning of the bag file, since there's no crossing of the finish line to start the lap.
This is due to the fact that, from the given map, the sector 1 starts from the line just after the pit entry of the track.
For the same reason, the Sector 3 is marked as finished when such line is crossed, and the (brief) final part of the bag records the car entering sector 1 before stopping completely.

The custom message type "sector_one" is given by the project details.
