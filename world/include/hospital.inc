define hospital model
(
  size [1 0.25 1]
  obstacle_return 0
  gui_move 0
  block
  (
    color "red"
    points [4]
    point[0] [1 0]
    point[1] [1 1]
    point[2] [2 1]
    point[3] [2 0]
    z [0 0.25]
  )
  block
  (
    color "red"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [3 1]
    point[3] [3 0]
    z [0.25 0.5]
  )
  block
  (
    color "red"
    points [4]
    point[0] [1 0]
    point[1] [1 1]
    point[2] [2 1]
    point[3] [2 0]
    z [0.5 0.75]
  )
)
