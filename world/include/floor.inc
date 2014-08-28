define wood-floor model
(
  size [20 20 0.01]
  obstacle_return 0
  gui_move 0
  block
  (
    color "tan"
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
    color "YellowGreen"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 0.5]
  )
)

define pink-flower model
(
  size [0.2 0.2 0.6]
  obstacle_return 0
  gui_move 0
  block
  (
    color "ForestGreen"
    points [4]
    point[0] [1.25 1.25]
    point[1] [1.25 1.75]
    point[2] [1.75 1.75]
    point[3] [1.75 1.25]
    z [0 0.8]
  )
  block
  (
    color "DeepPink"
    points [4]
    point[0] [0 0]
    point[1] [0 3]
    point[2] [3 3]
    point[3] [3 0]
    z [0.8 1]
  )
)

define blue-flower model
(
  size [0.2 0.2 0.6]
  obstacle_return 0
  gui_move 0
  block
  (
    color "ForestGreen"
    points [4]
    point[0] [1.25 1.25]
    point[1] [1.25 1.75]
    point[2] [1.75 1.75]
    point[3] [1.75 1.25]
    z [0 0.8]
  )
  block
  (
    color "LightSkyBlue"
    points [4]
    point[0] [0 0]
    point[1] [0 3]
    point[2] [3 3]
    point[3] [3 0]
    z [0.8 1]
  )
)

define white-flower model
(
  size [0.2 0.2 0.6]
  obstacle_return 0
  gui_move 0
  block
  (
    color "ForestGreen"
    points [4]
    point[0] [1.25 1.25]
    point[1] [1.25 1.75]
    point[2] [1.75 1.75]
    point[3] [1.75 1.25]
    z [0 0.8]
  )
  block
  (
    color "ivory"
    points [4]
    point[0] [0 0]
    point[1] [0 3]
    point[2] [3 3]
    point[3] [3 0]
    z [0.8 1]
  )
)


define tiles model
(
size [8.9 8.1 0.01]
  obstacle_return 0
  gui_move 0
  # first column #############
  block
  (
    color "snow"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [0 1]
    point[1] [0 2]
    point[2] [1 2]
    point[3] [1 1]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [0 2]
    point[1] [0 3]
    point[2] [1 3]
    point[3] [1 2]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [0 3]
    point[1] [0 4]
    point[2] [1 4]
    point[3] [1 3]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [0 4]
    point[1] [0 5]
    point[2] [1 5]
    point[3] [1 4]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [0 5]
    point[1] [0 6]
    point[2] [1 6]
    point[3] [1 5]
    z [0 0.5]
  )
  # second column #############
  block
  (
    color "snow"
    points [4]
    point[0] [1 0]
    point[1] [1 1]
    point[2] [2 1]
    point[3] [2 0]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [1 1]
    point[1] [1 2]
    point[2] [2 2]
    point[3] [2 1]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [1 2]
    point[1] [1 3]
    point[2] [2 3]
    point[3] [2 2]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [1 3]
    point[1] [1 4]
    point[2] [2 4]
    point[3] [2 3]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [1 4]
    point[1] [1 5]
    point[2] [2 5]
    point[3] [2 4]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [1 5]
    point[1] [1 6]
    point[2] [2 6]
    point[3] [2 5]
    z [0 0.5]
  )
  # third column #############
  block
  (
    color "snow"
    points [4]
    point[0] [2 0]
    point[1] [2 1]
    point[2] [3 1]
    point[3] [3 0]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [2 1]
    point[1] [2 2]
    point[2] [3 2]
    point[3] [3 1]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [2 2]
    point[1] [2 3]
    point[2] [3 3]
    point[3] [3 2]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [2 3]
    point[1] [2 4]
    point[2] [3 4]
    point[3] [3 3]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [2 4]
    point[1] [2 5]
    point[2] [3 5]
    point[3] [3 4]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [2 5]
    point[1] [2 6]
    point[2] [3 6]
    point[3] [3 5]
    z [0 0.5]
  )
  # fourth column #############
  block
  (
    color "snow"
    points [4]
    point[0] [3 0]
    point[1] [3 1]
    point[2] [4 1]
    point[3] [4 0]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [3 1]
    point[1] [3 2]
    point[2] [4 2]
    point[3] [4 1]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [3 2]
    point[1] [3 3]
    point[2] [4 3]
    point[3] [4 2]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [3 3]
    point[1] [3 4]
    point[2] [4 4]
    point[3] [4 3]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [3 4]
    point[1] [3 5]
    point[2] [4 5]
    point[3] [4 4]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [3 5]
    point[1] [3 6]
    point[2] [4 6]
    point[3] [4 5]
    z [0 0.5]
  )
  # fith column #############
  block
  (
    color "snow"
    points [4]
    point[0] [4 0]
    point[1] [4 1]
    point[2] [5 1]
    point[3] [5 0]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [4 1]
    point[1] [4 2]
    point[2] [5 2]
    point[3] [5 1]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [4 2]
    point[1] [4 3]
    point[2] [5 3]
    point[3] [5 2]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [4 3]
    point[1] [4 4]
    point[2] [5 4]
    point[3] [5 3]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [4 4]
    point[1] [4 5]
    point[2] [5 5]
    point[3] [5 4]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [4 5]
    point[1] [4 6]
    point[2] [5 6]
    point[3] [5 5]
    z [0 0.5]
  )
  # sixth column #############
  block
  (
    color "snow"
    points [4]
    point[0] [5 0]
    point[1] [5 1]
    point[2] [6 1]
    point[3] [6 0]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [5 1]
    point[1] [5 2]
    point[2] [6 2]
    point[3] [6 1]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [5 2]
    point[1] [5 3]
    point[2] [6 3]
    point[3] [6 2]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [5 3]
    point[1] [5 4]
    point[2] [6 4]
    point[3] [6 3]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [5 4]
    point[1] [5 5]
    point[2] [6 5]
    point[3] [6 4]
    z [0 0.5]
  )
  block
  (
    color "snow"
    points [4]
    point[0] [5 5]
    point[1] [5 6]
    point[2] [6 6]
    point[3] [6 5]
    z [0 0.5]
  )
)



