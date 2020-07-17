*****
AUVKit
*****

What is it?
########
AUVKit is a Python library for communicating, controlling and writing apps for AUVs (Autonomous underwater vehicles).
AUVKit currently support the BlueRobotics BlueROV2. We are currently looking for contributors to contribute to this
project. AUVKit is an open source project that was created and designed by the Cyber Physical Systems Laboratory
at Rutgers, the State University of New Jersey in  New Brunswick, NJ, USA.

What can it do? How do I get started?
########
AUVKit aims to simplify the creation of applications for AUVs for developers so they do not have to focus on unnecessary
error handling and motor contor. The API is designed to be easy to use yet powerful. AUVKit supports GPS navigation
or alternatively, supports movement given a displacement. AUVKit also supports writing data to a CSV for data
collection. AUVKit currently supports data retrieval from Atlas Scientific Sensors and from MAVLink messages. AUVKit
also currently only supports the BlueRobotics Ping Echosonar.

To get started, check out the examples in the ``examples/`` folder in this repo.

Future Work
########
Currently, we would like to add dynamic PID control to the library. We would also like to implement more sophisticated
Kalman filters for localization techniques. Also, we would like to add an API for acoustic communication modems.

Acknowledgements
**********************

This project was based off the work of Wahhaj Zahedi, Milos Seskar, Agam Modasiya, Karun Kanda, Archana Arjula,
and Mohammad Nadeem under the guidance Mehdi Rahmati.