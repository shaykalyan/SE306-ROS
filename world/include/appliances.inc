define appliances model
(
# sombre, sensible, artistic
color "Black"
# most maps will need a bounding box
boundary 0
gui_move 0
ranger_return 1
)

define shower model
(
  size [1.5 1.5 3]
  obstacle_return 0
  block
  (
    color "GhostWhite"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 0.1]
  )
  block
  (
    color "GhostWhite"
    points [4]
    point[0] [0 0]
    point[1] [0 0.01]
    point[2] [0.01 0.01]
    point[3] [0.01 0]
    z [0.1 2.9]
  )
  block
  (
    color "GhostWhite"
    points [4]
    point[0] [0 0.99]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0.99]
    z [0.1 2.9]
  )
  block
  (
    color "GhostWhite"
    points [4]
    point[0] [1 1]
    point[1] [1 0]
    point[2] [0.99 0]
    point[3] [0.99 1]
    z [0.1 2.9]
  )
  block
  (
    color "GhostWhite"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [2.9 3]
  )
)

