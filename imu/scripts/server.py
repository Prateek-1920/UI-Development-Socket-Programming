import socket
import rospy
import math
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


HOST = '192.168.52.95'  # IP OF THE NETWORK CONNECTED TO
PORT = 8907  # SAME ON BOTH THE DEVICES


def euler_to_quaternion(roll, pitch, yaw):

    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return quaternion


def socket_callback():
    rospy.init_node('imu_pub', anonymous=True)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))

    server_socket.listen(1)
    print("Server listening on {}:{}".format(HOST, PORT))

    client_socket, client_address = server_socket.accept()
    print("Connected to client:", client_address)

    imu_pub = rospy.Publisher('/phone_imu', Imu, queue_size=10)

    rate = rospy.Rate(10)
    client_socket.settimeout(5)

    while not rospy.is_shutdown():
        try:
            data = client_socket.recv(1024)
            if not data:
                raise socket.timeout("No data recieved")
        except socket.timeout:
            # EXIT IF CONNECTION  IS TERMINATED
            print("Timeout occurred. No data received.")
            break

        data1 = data.decode("utf-8")  # BYTE TO STRING DECODING USING UTF-8
        data1 = data1.replace("\r", "").replace("\n", "")
        values = data1.split(',')

        try:
            values.remove('')  # REMOVE EMPTY VALUES
        except ValueError:
            pass

        if len(values) == 9:  # NECESSARY 9 VALUES
            print(values)

            imu_msg = Imu()

            imu_msg.header.frame_id = 'map'

            imu_msg.angular_velocity.x = float(values[0])
            imu_msg.angular_velocity.y = float(values[1])
            imu_msg.angular_velocity.z = float(values[2])

            imu_msg.linear_acceleration.x = float(values[3])
            imu_msg.linear_acceleration.y = float(values[4])
            imu_msg.linear_acceleration.z = float(values[5])

            w, x, y, z = euler_to_quaternion(
                float(values[6]), float(values[7]), float(values[8]))

            imu_msg.orientation.x = x
            imu_msg.orientation.y = y
            imu_msg.orientation.z = z
            imu_msg.orientation.w = w

            imu_pub.publish(imu_msg)

        else:
            continue
        print(f'Received IMU data: {data1}')

    # Close the client socket
    client_socket.close()

    # Close the server socket
    server_socket.close()


if __name__ == '__main__':

    print("##### SERVER NODE #####")
    socket_callback()
