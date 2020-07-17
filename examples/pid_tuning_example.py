from auvkit.path import Path, PathNode

# Create a Path Object
path = Path()

# Tune the heading PID
path.set_heading_pid(kP=0.3, kI=0.4, kD=0.5)

# Tune the vertical PID
path.set_vertical_pid(kP=0.3, kI=0.4, kD=0.5)

# Tune the turning PID
path.set_turning_pid(kP=0.3, kI=0.4, kD=0.5)
