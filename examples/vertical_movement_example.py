from auvkit.path import Path, PathNode

# Create a Path Object
path = Path()

# Enable stabilize mode (optional)
path.set_stabilize_mode()

# Enable depth hold mode (optional but recommended)
path.set_depth_hold()

# Move 3 meter downs.
# Then move 1 meter up.
path.push(PathNode(0, 0, 1))
path.push(PathNode(0, 0, 1))
path.move_entire_route()
