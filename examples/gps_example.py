from auvkit.path import Path

# Create a Path Object
path = Path()

# Enable GPS
path.enable_gps()

# Make AUV go to certain GPS coordinates. Example given is GPS coordinate (-45, 74)
path.go_to(-45, 74)
