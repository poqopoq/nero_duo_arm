std_msgs/Header header

# Logical name configured for this sensor, for example "left".
string sensor_id

# Hardware serial number read from the sensor with command 0x04.
# Empty when the sensor has not answered the identity query yet.
string serial_number

# Six force/torque readings in the order received from the sensor.
# The V0.1.0 UART protocol does not define channel order, sign or units.
float32[6] data
