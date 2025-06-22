define laser ranger
(
  sensor
  (
    range_max 20.0
    fov 360.0
    samples 1080
  )
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  odom_error [0.01 0.01 0.0 0.0 0.0 0.01]

  size [ 0.54 0.40 0.15 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "red"
  
  laser(pose [ 0.0 0.0 0.0 0.0 ])
  
  # Velocity bounds: [xmin xmax ymin ymax zmin zmax amin amax]
  velocity_bounds [-0.5 0.5 -0.5 0.5 0.0 0.0 -90.0 90.0]
  acceleration_bounds [-0.5 0.5 -0.5 0.5 0.0 0.0 -90.0 90.0]
)