from auvkit.path import Path

# Create a Path Object
path = Path()

# Get the data fetcher from the path
data_fetcher = path.data_fetcher

# Read the temperature from the data fetcher
data_fetcher.get_temp()

# You can read other values as well, see docs for details
