import rclpy
from rclpy.node import Node
from sensor_msgs.msg import TimeReference
from rclpy.qos import QoSProfile, ReliabilityPolicy
import ctypes
import time
import sysv_ipc
import threading
import subprocess
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

# Structure as defined in chrony sources (see chrony/chrony.h)
class shmTime(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_int),
        ("count", ctypes.c_int),
        ("clockTimeStampSec", ctypes.c_long),  
        ("clockTimeStampUSec", ctypes.c_int),
        ("receiveTimeStampSec", ctypes.c_long),  
        ("receiveTimeStampUSec", ctypes.c_int),
        ("leap", ctypes.c_int),
        ("precision", ctypes.c_int),
        ("nsamples", ctypes.c_int),
        ("valid", ctypes.c_int),
        ("dummy", ctypes.c_int * 10)
    ]

class TimeToSHM(Node):
    def __init__(self):
        super().__init__('time_to_shm')
        self.get_logger().debug("Initializing TimeToSHM node")
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.subscription = self.create_subscription(
            TimeReference,
            '/mavros/time_reference',
            self.listener_callback,
            qos_profile)
        self.shm_key = 0x4e545030  # "NTP0"
        self.shm_size = ctypes.sizeof(shmTime)
        self.get_logger().debug(f"SHM key: {hex(self.shm_key)}, SHM size: {self.shm_size}")
        self.attach_shm()

        self._shutdown = False
        self.monitor_thread = threading.Thread(target=self.monitor_chrony, daemon=True)
        self.monitor_thread.start()

        self.diagnostic_pub = self.create_publisher(DiagnosticStatus, '/time_sync/diagnostics', 10)

    def attach_shm(self):
        try:
            self.get_logger().debug("Trying to create shared memory segment...")
            self.shm = sysv_ipc.SharedMemory(self.shm_key, sysv_ipc.IPC_CREAT, size=self.shm_size, mode=0o666)
            self.get_logger().debug("Shared memory segment created.")
        except sysv_ipc.PermissionsError:
            self.get_logger().debug("Permissions error: removing existing segment and retrying...")
            try:
                existing = sysv_ipc.SharedMemory(self.shm_key)
                existing.remove()
                self.get_logger().debug("Removed existing shared memory segment.")
                self.shm = sysv_ipc.SharedMemory(self.shm_key, sysv_ipc.IPC_CREAT, size=self.shm_size, mode=0o666)
                self.get_logger().debug("Shared memory segment created after removal.")
            except Exception as e:
                self.get_logger().error(f"Failed to remove and recreate SHM: {e}")
                raise
        except sysv_ipc.ExistentialError:
            self.get_logger().debug("Shared memory segment exists, attaching...")
            self.shm = sysv_ipc.SharedMemory(self.shm_key)
            self.get_logger().debug("Attached to existing shared memory segment.")

    def listener_callback(self, msg):
        self.get_logger().debug("Received TimeReference message")
        sec = msg.time_ref.sec
        nsec = msg.time_ref.nanosec
        now = time.time()
        st = shmTime()
        st.mode = 0
        st.count = 0
        st.clockTimeStampSec = sec
        st.clockTimeStampUSec = int(nsec / 1000)
        st.receiveTimeStampSec = int(now)
        st.receiveTimeStampUSec = int((now - int(now)) * 1e6)
        st.leap = 0
        st.precision = -1
        st.nsamples = 3

        # Synchronization: set valid=0, increment count, write, set valid=1, increment count, write again
        st.valid = 0
        st.count += 1
        self.get_logger().debug(f"Writing to SHM with valid=0, count={st.count}")
        self.shm.write(bytes(st))
        st.valid = 1
        st.count += 1
        self.get_logger().debug(f"Writing to SHM with valid=1, count={st.count}")
        self.shm.write(bytes(st))
        self.get_logger().info(f"Published time {sec}.{nsec} to chrony SHM")

    def publish_time_sync_diagnostic(self, synced, message):
        status = DiagnosticStatus()
        status.name = "PX4 Time Sync"
        status.hardware_id = "chrony_shm"
        if synced:
            status.level = DiagnosticStatus.OK
            status.message = "PX4 time source synchronized"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "PX4 time source NOT synchronized"
        self.diagnostic_pub.publish(status)

    #You need to configure passwordless sudo for this to work without prompting for password
    #1. Open a terminal and run `sudo visudo`
    #2. Add the following line at the end of the file: yourusername ALL=NOPASSWD: /bin/systemctl restart chrony
    def monitor_chrony(self):
        while not self._shutdown:
            try:
                result = subprocess.run(
                    ["chronyc", "sources", "-v"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=5
                )
                output = result.stdout
                self.get_logger().info("\nOutput from chronyc sources:" + output)
                synced = any(line.startswith('#*') and 'PX4' in line for line in output.splitlines())
                if not synced:
                    self.get_logger().warn("PX4 time source not synchronized! Restarting chrony...")
                    self.publish_time_sync_diagnostic(False, synced)
                    subprocess.run(["sudo", "systemctl", "restart", "chrony"]) 
                else:
                    self.get_logger().info("PX4 time source is synchronized.")
                    self.publish_time_sync_diagnostic(True, synced)
                time.sleep(30)
            except Exception as e:
                self.get_logger().error(f"Error monitoring chrony: {e}")
                self.publish_time_sync_diagnostic(False, str(e))
                time.sleep(30)

    def shutdown(self):
        self.get_logger().info("Shutting down TimeToSHM node...")
        self._shutdown = True
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5)
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TimeToSHM()
    node.get_logger().debug("Starting main()")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Exiting...")
    finally:
        node.shutdown()  # This destroys the node and joins the thread
        rclpy.shutdown() # Shutdown ROS context last, no logging after this

if __name__ == '__main__':
    main()