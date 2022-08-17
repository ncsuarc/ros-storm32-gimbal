import rclpy
from rclpy.node import Node
import tf_transformations as tf
from .storm32 import Storm32
import numpy as np
import serial
import diagnostic_updater
import diagnostic_msgs.msg
from arcros_interface.msg import GimbalOrientation
from std_srvs.srv import Trigger
from geometry_msgs.msg import QuaternionStamped, Quaternion


class Storm32Node(Node):
    def __init__(self):
        # Init the ROS node
        super().__init__("control")

        # Getting ROS params
        self.declare_parameter("~/port", "/dev/ttyACM0")
        self.declare_parameter("~/frame_id", "gimbal_ref")
        self.port = self.get_parameter("~/port").get_parameter_value().string_value
        self.frame_id = self.get_parameter("~/frame_id").get_parameter_value().string_value

        # Start gimbal object
        self.gimbal = Storm32(port=self.port)
        version = self.gimbal.get_version()

        # Print out some diagnostic info
        if version:
            self.get_logger().info("Gimbal found at {0}".format(self.port))
            for k, v in version.items():
                self.get_logger().info("{0}: {1}".format(k, v))

            # Setup the diagnostic updater
            self.updater = diagnostic_updater.Updater()
            self.updater.setHardwareID(self.frame_id)
            self.updater.add("Gimbal Diagnostics", self.get_diagnostics_status)

            # Setup subscriber and publisher
            self.sub = self.create_subscription(
                GimbalOrientation,
                "~/target_orientation",
                self.gimbal_quaternion_callback,
                queue_size=1)
            self.camera_pub = self.create_publisher(
                QuaternionStamped, "~/camera_orientation", queue_size=1)
            self.controller_pub = self.create_publisher(
                QuaternionStamped, "~/controller_orientation", queue_size=1)
            self.restart_srv = self.create_service(Trigger, "~/restart", self.restart_controller)

            # Setup periodic callback for orientation publisher
            self.pub_timer = self.create_timer(0.01, self.pub_timer_callback)
        else:
            rclpy.get_logger().fatal("Gimbal unresponsive at {0}".format(self.port))
            rclpy.shutdown()

    def restart_shutdown_callback(self, event):
        """Restart callback used to make sure service is able to respond before
        shutting down.

        Args:
            event: Timer callback event.
        """
        self.restart_timer.destroy() # Ensure the timer doesn't call this method again.
        self.getLogger().warning("Gimbal restarted, node shutting down!")
        rclpy.shutdown("Gimbal restarted, node shutting down!")


    def encode_angle(self, x):
        """Convert radian angle into degree within +-180 degree.

        Args:
            x: Radian angle to be converted.

        Returns: Degree angle within +-180 degree.
        """
        while x > np.pi:
            x -= 2 * np.pi
        while x < -np.pi:
            x += 2 * np.pi
        return x * 180 / np.pi


    def gimbal_quaternion_callback(self, msg):
        """Callback function for setting target_orientation. This function will
        terminate the ROS node if it encounters a SerialException.

        Arg:
            msg: target_orientation message.
        """
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z,
                    msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion, axes="syxz")
        euler = list(map(self.encode_angle, euler))
        self.get_logger().info(
            "set_angles : pitch={0}, roll={1}, yaw={2}, unlimited={3}".format(
                euler[0], euler[1], euler[2], msg.unlimited))
        try:
            response = self.gimbal.set_angles(*euler, unlimited=msg.unlimited)
            if response != [1, 150, 0]:
                self.get_logger().error("STorM32: set_pitch_roll_yaw:" +
                            " response error : {0}:s".format(response))
        except serial.serialutil.SerialException as e:
            self.get_logger().fatal(e)
            rclpy.shutdown(f"SerialError: {e}")


    def get_diagnostics_status(self, stat):
        """Callback function for periodically update diagnostics. This function
        will terminate the ROS node if it encounters a SerialException.

        Arg:
            stat: Status object required for diagnostic_updater.

        Returns: Updated status object.
        """
        # Get the status
        status = self.gimbal.get_status()

        # Put all status into stat object
        if status:
            for k, v in status.items():
                stat.add(k, v)
            self.get_logger().info("Gimbal States: {0}".format(status["State"]))
            if status["Battery Connected"]:
                self.get_logger().info("VBAT={0}, Voltage {1}".format(
                    status["VBAT"], "low" if status["Bat Voltage Low"] else "OK"))
            else:
                self.get_logger().warn("Battery disconnected!")
            # Put the state of the gimbal into diagnostic summary
            state = status["State"]
            if state == "Normal":
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, state)
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, state)
        return stat


    def restart_controller(self, req):
        """Callback function to restart the gimbal controller. This function will
        terminate the ROS node if it encounters a SerialException.

        Arg:
            req: Request object for the ROS Service.

        Returns: Service Response, "True" for success, "False" for error.
        """
        try:
            response = Trigger()
            success = self.gimbal.restart_controller()
            response.success = success
            if success:
                response.message = "Gimbal restarted successfully!"

                # Shutdown the node on success restart
                self.sub.unregister()
                self.pub_timer.shutdown()
                self.camera_pub.unregister()
                self.controller_pub.unregister()
                self.restart_timer = self.create_timer(0.1, self.restart_shutdown_callback)
            else:
                response.message = "Gimbal restart failed!"
            return response

        except serial.serialutil.SerialException as e:
            self.get_logger().fatal(e)
            rclpy.shutdown(f"SerialError: {e}")


    def pub_timer_callback(self, event):
        """Periodic callback function to publish the current orientation of the
        gimbal. This function  will terminate the ROS node if it encounters a
        SerialException.

        Arg:
            event: ROS event object for the periodic callback.
        """
        # Create new message and header
        msg = QuaternionStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now()

        try:
            imu1 = (self.gimbal.get_imu1_angles(), self.camera_pub, "imu1")
            imu2 = (self.gimbal.get_imu2_angles(), self.controller_pub, "imu2")
            for euler, pub, name in (imu1, imu2):
                if euler:
                    self.get_logger().debug(
                        f"get_{name}_angles: pitch={euler[0]:.2f}, roll={euler[2]:.2f}, yaw={euler[3]:.2f}",
                        throttle_duration_sec=1.0)
                    euler = list(map(np.radians, euler))
                    q = tf.transformations.quaternion_from_euler(
                        *euler, axes="syxz")
                    msg.quaternion.x = q[0]
                    msg.quaternion.y = q[1]
                    msg.quaternion.z = q[2]
                    msg.quaternion.w = q[3]
                    pub.publish(msg)

            # diagnostic update is called here because this run at a high rate
            self.updater.update()
        except serial.serialutil.SerialException as e:
            self.get_logger().fatal(e)
            rclpy.shutdown(f"SerialError: {e}")

def main():
    rclpy.init()
    rclpy.spin(Storm32Node())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
