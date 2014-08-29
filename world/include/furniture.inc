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

define bench model
(
  size [4 1.5 1.25]
  obstacle_return 0
  gui_move 0
  block
  (
    color "PeachPuff"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 0.5]
  )
  block
  (
    color "gainsboro"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0.5 0.55]
  )
)

define table model
(
  size [2.5 2.5 1.25]
  obstacle_return 0
  gui_move 0
  block
  (
    color "peru"
    points [4]
    point[0] [0 0]
    point[1] [0 0.05]
    point[2] [0.05 0.05]
    point[3] [0.05 0]
    z [0 1.2]
  )
  block
  (
    color "peru"
    points [4]
    point[0] [0 0.95]
    point[1] [0 1]
    point[2] [0.05 1]
    point[3] [0.05 0.95]
    z [0 1.2]
  )
  block
  (
    color "peru"
    points [4]
    point[0] [0.95 0.95]
    point[1] [0.95 1]
    point[2] [1 1]
    point[3] [1 0.95]
    z [0 1.2]
  )
  block
  (
    color "peru"
    points [4]
    point[0] [0.95 0]
    point[1] [0.95 0.05]
    point[2] [1 0.05]
    point[3] [1 0]
    z [0 1.2]
  )
  block
  (
    color "peru"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [1.2 1.25]
  )
)

define couch model
(
  size [3.5 1.5 1.25]
  obstacle_return 0
  gui_move 0
  block
  (
    color "PaleTurquoise"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 1.25]
  )
  block
  (
    color "PaleTurquoise"
    points [4]
    point[0] [1 0]
    point[1] [1 1]
    point[2] [5 1]
    point[3] [5 0]
    z [0 0.3]
  )
  block
  (
    color "PaleTurquoise"
    points [4]
    point[0] [5 0]
    point[1] [5 1]
    point[2] [6 1]
    point[3] [6 0]
    z [0 1.25]
  )
  block
  (
    color "PaleTurquoise"
    points [4]
    point[0] [1 0]
    point[1] [1 0.25]
    point[2] [5 0.25]
    point[3] [5 0]
    z [0.3 1.25]
  )
  block
  (
    color "PaleTurquoise"
    points [4]
    point[0] [1 0.25]
    point[1] [1 1]
    point[2] [2.95 1]
    point[3] [2.95 0.25]
    z [0.3 0.7]
  )
  block
  (
    color "PaleTurquoise"
    points [4]
    point[0] [3.05 0.25]
    point[1] [3.05 1]
    point[2] [5 1]
    point[3] [5 0.25]
    z [0.3 0.7]
  )
)

define small-table model
(
  size [2.5 1 1.25]
  obstacle_return 0
  gui_move 0
  block
  (
    color "peru"
    points [4]
    point[0] [0 0]
    point[1] [0 0.05]
    point[2] [0.05 0.05]
    point[3] [0.05 0]
    z [0 1.2]
  )
  block
  (
    color "peru"
    points [4]
    point[0] [0 0.95]
    point[1] [0 1]
    point[2] [0.05 1]
    point[3] [0.05 0.95]
    z [0 1.2]
  )
  block
  (
    color "peru"
    points [4]
    point[0] [0.95 0.95]
    point[1] [0.95 1]
    point[2] [1 1]
    point[3] [1 0.95]
    z [0 1.2]
  )
  block
  (
    color "peru"
    points [4]
    point[0] [0.95 0]
    point[1] [0.95 0.05]
    point[2] [1 0.05]
    point[3] [1 0]
    z [0 1.2]
  )
  block
  (
    color "peru"
    points [4]
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [1.2 1.25]
  )
)
