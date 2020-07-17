from auvkit.path import Path
from auvkit.data_fetcher import AtlasScientificSensor

# Create a Path Object
path = Path()

# Create an AtlasScientificSensor object and specify an identifier and the corresponding serial port
# Serial port varies based on configuration, /dev/ttyUSB1 is an example.
conductivity = AtlasScientificSensor(identifier="conductivity", serial_port="/dev/ttyUSB1")

# Add the sensor to the path.
path.add_atlas_sensor_object(conductivity)

# Alternatively, you can add a sensor by passing in the identifier and serial port.
path.add_atlas_sensor(identifier="conductivity", serial_port="/dev/ttyUSB1")

# Read from the sensor
value_read = conductivity.read_sensor()
