define bed model
(
  size [2 3 1.25]
  obstacle_return 0
  gui_move 0
  block
  (
    color "PeachPuff"
    points [4]
    point[0] [0 0]
    point[1] [0 3]
    point[2] [2 3]
    point[3] [2 0]
    z [0 0.5]
  )
  block
  (
    color "CornflowerBlue"
    points [4]
    point[0] [0 1]
    point[1] [0 3]
    point[2] [2 3]
    point[3] [2 1]
    z [0.5 1]
  )
  block
  (
    color "white"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [2 1]
    point[3] [2 0]
    z [0.5 1]
  )
  block
  (
    color "CornflowerBlue"
    points [4]
    point[0] [0.05 0.05]
    point[1] [0.05 0.70]
    point[2] [0.95 0.70]
    point[3] [0.95 0.05]
    z [1 1.2]
  )
  block
  (
    color "CornflowerBlue"
    points [4]
    point[0] [1.05 0.05]
    point[1] [1.05 0.70]
    point[2] [1.95 0.70]
    point[3] [1.95 0.05]
    z [1 1.2]
  )
)
