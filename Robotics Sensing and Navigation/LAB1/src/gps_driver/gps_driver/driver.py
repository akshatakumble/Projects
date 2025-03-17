import serial
import sys
import utm
import rclpy #ROS2 Python client library
from rclpy.node import Node #Base class for ROS2 nodes
from std_msgs.msg import Header #Standard ROS message header
from gps_msgs.msg import GPSmsg #Custom GPS message type
from builtin_interfaces.msg import Time


#Converts latitude from degrees and minutes format to decimal degrees.
def convert_to_decimal(degrees_minutes, direction):
    try:
        degrees = int(degrees_minutes[:2])	#Extract degrees (first 2 characters)
        minutes = float(degrees_minutes[2:])	#Extract minutes (rest of the string)
        decimal_degrees = degrees + (minutes / 60)	#Convert to decimal format
        return -decimal_degrees if direction in ['S', 'W'] else decimal_degrees	#Adjust sign based on direction
    except ValueError:
        return None
        
        
#Converts longitude from degrees and minutes format to decimal degrees.        
def convert_to_decimalLon(degrees_minutes, direction):
    try:
        degrees = int(degrees_minutes[:3])
        minutes = float(degrees_minutes[3:])
        decimal_degrees = degrees + (minutes / 60)
        return -decimal_degrees if direction in ['S', 'W'] else decimal_degrees
    except ValueError:
        return None

#Parses an NMEA $GPGGA sentence and extracts time, latitude, longitude, altitude, and HDOP.
def parse_gpgga(gpgga_sentence):
    parts = gpgga_sentence.split(',')
    if gpgga_sentence.startswith('$GPGGA') and len(parts) > 14:
        time_utc = parts[1]
        latitude = convert_to_decimal(parts[2], parts[3])
        longitude = convert_to_decimalLon(parts[4], parts[5])
        altitude = float(parts[9]) if parts[9] else None
        hdop = float(parts[8])
        return (time_utc, latitude, longitude, altitude, hdop) if all((latitude, longitude)) else None
    return None

#ROS2 node that reads GPS data from a serial port, parses it, and publishes it as a ROS message.
class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyACM0'),
                ('baudrate', 4800),
                ('sampling_rate', 10.0),
                ('timeout', 3.0)
            ]
        )

        custom_port = self.parse_command_line_args()
        self.port_name = custom_port or self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baudrate').value
        self.sampling_rate = self.get_parameter('sampling_rate').value
        self.timeout = self.get_parameter('timeout').value

	# Initialize the serial port
        self.initialize_serial_port()
        
        # Create a ROS2 publisher for GPS messages
        self.publisher_ = self.create_publisher(GPSmsg, 'gps', 10)
        self.timer = self.create_timer(1.0 / self.sampling_rate, self.timer_callback)
        self.last_gpgga_time = self.get_clock().now()

#Parses command-line arguments to override default serial port.
    def parse_command_line_args(self):
        return next((arg.split(':')[1] for arg in sys.argv if arg.startswith('port:')), None)
        
#Attempts to open the serial port.
    def initialize_serial_port(self):
        try:
            self.port = serial.Serial(self.port_name, self.baud_rate, timeout=self.timeout)
            self.get_logger().info(f"Connected to GPS device on {self.port_name} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {self.port_name}: {e}")
            self.port = None
            rclpy.shutdown()

#Reads data from the GPS and processes $GPGGA sentences.
    def timer_callback(self):
        if not self.port:
            self.get_logger().warn("Serial port is not initialized.")
            return

        try:
            line = self.port.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA'):
                self.process_gpgga(line)
            self.check_gpgga_timeout()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

#Processes a received $GPGGA sentence and publishes GPS data.
    def process_gpgga(self, line):
        self.get_logger().info(f"Raw GPS Data: {line}")
        parsed_data = parse_gpgga(line)
        if parsed_data:
            self.publish_gps_data(*parsed_data)
            self.last_gpgga_time = self.get_clock().now()
            
#Publishes GPS data as a ROS2 message.
    def publish_gps_data(self, time_utc, latitude, longitude, altitude, hdop):
        utm_coords = utm.from_latlon(latitude, longitude)
        msg = GPSmsg()
        msg.header = Header()
        msg.header.stamp = self.create_gps_time(time_utc)
        msg.header.frame_id = "GPS1_FRAME"
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude if altitude is not None else float('nan')
        msg.hdop = hdop
        msg.utm_easting, msg.utm_northing, msg.zone, msg.letter = utm_coords
        

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Data: Lat: {latitude}, Lon: {longitude}, Alt: {altitude}, HDOP: {hdop}")

#Converts UTC time from GPGGA to ROS2 Time format.
    def create_gps_time(self, time_utc):
        hours, minutes, seconds = int(time_utc[0:2]), int(time_utc[2:4]), float(time_utc[4:])
        gps_time = Time()
        gps_time.sec = int(hours * 3600 + minutes * 60 + int(seconds))
        gps_time.nanosec = int((seconds % 1) * 1e9)
        return gps_time

#Warns if no GPGGA sentence is received for over 5 seconds.
    def check_gpgga_timeout(self):
        if (self.get_clock().now() - self.last_gpgga_time).nanoseconds > 5e9:
            self.get_logger().warn("No $GPGGA sentence received for over 5 seconds.")


#Main function to initialize and run the GPS publisher node.
def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    
    if gps_publisher.port:
        try:
            rclpy.spin(gps_publisher)
        except KeyboardInterrupt:
            gps_publisher.get_logger().info("Node stopped cleanly.")
        finally:
            gps_publisher.destroy_node()
            rclpy.shutdown()
    else:
        gps_publisher.get_logger().error("GPS publisher failed to start due to serial port error.")

if __name__ == '__main__':
    main()
