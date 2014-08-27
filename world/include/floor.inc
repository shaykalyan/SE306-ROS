define wood-floor model
(
  size [20 20 0.01]
  obstacle_return 0
  gui_move 0
  block
  (
    color "sienna"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 0.5]
  )
)

define grass model
(
  size [7.5 3 0.01]
  obstacle_return 0
  gui_move 0
  block
  (
    color "LimeGreen"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 0.5]
  )
)
