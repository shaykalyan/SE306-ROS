define hospital model
(
  size [1.5 1.5 0.1]
  obstacle_return 0
  gui_move 0
  block
  (
    color "red"
    points [4]
    point[0] [0 1]
    point[1] [0 2]
    point[2] [3 2]
    point[3] [3 1]
    z [0 1]
  )
  block
  (
    color "red"
    points [4]
    point[0] [1 0]
    point[1] [1 3]
    point[2] [2 3]
    point[3] [2 0]
    z [0 1]
  )
#  block
#  (
#    color "red"
#    points [4]
#    point[0] [1 0]
#    point[1] [1 1]
#    point[2] [2 1]
#    point[3] [2 0]
#    z [0.5 0.75]
#  )
)
