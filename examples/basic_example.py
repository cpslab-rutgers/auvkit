from auvkit.path import Path, PathNode

# Create a Path Object
path = Path()

# Enable stabilize mode (optional)
path.set_stabilize_mode()

# Move 5 meters at an angle change of 45 degrees.
# Then move, 10 meters at an angle change of 90 degrees.
path.push(PathNode(5, 45, 0))
path.push(PathNode(10, 90, 0))
path.move_entire_route()

# Alternatively, we can move one node at a time instead of through queues.
path.move_one_node(PathNode(5, 135, 0))
