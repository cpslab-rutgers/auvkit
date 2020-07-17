from auvkit.path import Path, PathNode

# Create a Path Object
path = Path()

# Call function to write to CSV
path.write_to_csv()

# Can also specify filename if needed
path.write_to_csv(filename="example.csv")

# Move 5 meters at an angle change of 45 degrees.
# Then move, 10 meters at an angle change of 90 degrees.
path.push(PathNode(5, 45, 0))
path.push(PathNode(10, 90, 0))
path.move_entire_route()
