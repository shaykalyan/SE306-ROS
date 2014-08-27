define shower model
(
  size [1.5 1.5 3]
  obstacle_return 0
  gui_move 0
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

define oven model
(
  size [1.5 1.5 1.5]
  obstacle_return 0
  gui_move 0
  block
  (
    color "OrangeRed1"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 0.9]
  )
  block
  (
    color "OrangeRed1"
    points [4]
    point[0] [0 0.95]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0.95]
    z [0.9 1.1]
  )
  block
  (
    color "black"
    points [4]
    point[0] [0.125 0.125]
    point[1] [0.125 0.425]
    point[2] [0.425 0.425]
    point[3] [0.425 0.125]
    z [0.9 0.92]
  )
  block
  (
    color "black"
    points [4]
    point[0] [0.575 0.125]
    point[1] [0.575 0.425]
    point[2] [0.875 0.425]
    point[3] [0.875 0.125]
    z [0.9 0.92]
  )
  block
  (
    color "black"
    points [4]
    point[0] [0.575 0.575]
    point[1] [0.575 0.875]
    point[2] [0.875 0.875]
    point[3] [0.875 0.575]
    z [0.9 0.92]
  )
  block
  (
    color "black"
    points [4]
    point[0] [0.125 0.575]
    point[1] [0.125 0.875]
    point[2] [0.425 0.875]
    point[3] [0.425 0.575]
    z [0.9 0.92]
  )
)

define toilet model
(
  size [1.5 1.25 1.5]
  obstacle_return 0
  gui_move 0
  block
  (
    color "azure"
    points [4]
    point[0] [0 1]
    point[1] [0 2]
    point[2] [3 2]
    point[3] [3 1]
    z [0 0.3]
  )
  block
  (
    color "azure"
    points [4]
    point[0] [0 0.5]
    point[1] [0 2.5]
    point[2] [3.25 2.5]
    point[3] [3.25 0.5]
    z [0.3 0.7]
  )
  block
  (
    color "azure"
    points [4]
    point[0] [0 0]
    point[1] [0 3]
    point[2] [1.25 3]
    point[3] [1.25 0]
    z [0.7 1.4]
  )
  block
  (
    color "azure"
    points [4]
    point[0] [1.25 0.5]
    point[1] [1.25 2.5]
    point[2] [3.25 2.5]
    point[3] [3.25 0.5]
    z [0.7 0.8]
  )
)

define sink model
(
  size [1 1.5 1.6]
  obstacle_return 0
  gui_move 0
  block
  (
    color "azure"
    points [4]
    point[0] [0 1.5]
    point[1] [0 2.5]
    point[2] [1 2.5]
    point[3] [1 1.5]
    z [0 0.9]
  )
  block
  (
    color "azure"
    points [4]
    point[0] [0 0]
    point[1] [0 4]
    point[2] [2 4]
    point[3] [2 0]
    z [0.9 1.2]
  )
  block
  (
    color "grey"
    points [4]
    point[0] [0 2.5]
    point[1] [0 3]
    point[2] [0.5 3]
    point[3] [0.5 2.5]
    z [1.2 1.3]
  )
  block
  (
    color "grey"
    points [4]
    point[0] [0 2.5]
    point[1] [0 3]
    point[2] [1 3]
    point[3] [1 2.5]
    z [1.3 1.4]
  )
  block
  (
    color "grey"
    points [4]
    point[0] [0 1]
    point[1] [0 1.5]
    point[2] [0.5 1.5]
    point[3] [0.5 1]
    z [1.2 1.3]
  )
  block
  (
    color "grey"
    points [4]
    point[0] [0 1]
    point[1] [0 1.5]
    point[2] [1 1.5]
    point[3] [1 1]
    z [1.3 1.4]
  )
)

