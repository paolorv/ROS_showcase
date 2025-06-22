define laser ranger
(
  sensor
  (
    range_max 100.0
    fov 360.0
    samples 1080
  )
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define robot position
(
  localization "gps"
  localization_origin [ 0 0 0 0 ]
  
  size [ 0.54 0.40 0.15 ]
  origin [ 0 0 0 0 ]
  gui_nose 1
  drive "diff"
  color "red"
  
  laser(pose [ 0 0 0 0 ])
  
  velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]
)