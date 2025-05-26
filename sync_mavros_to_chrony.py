import rclpy
from rclpy.node import Node
from sensor_msgs.msg import TimeReference
from rclpy.qos import QoSProfile, ReliabilityPolicy
import ctypes
import time
import sysv_ipc

# Structure as defined in chrony sources (see chrony/chrony.h)
class shmTime(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_int),
        ("count", ctypes.c_int),
        ("clockTimeStampSec", ctypes.c_long),  # changed from c_time_t
        ("clockTimeStampUSec", ctypes.c_int),
        ("receiveTimeStampSec", ctypes.c_long),  # changed from c_time_t
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
        print("[DEBUG] Initializing TimeToSHM node")
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
        print(f"[DEBUG] SHM key: {hex(self.shm_key)}, SHM size: {self.shm_size}")
        self.attach_shm()

    def attach_shm(self):
        try:
            print("[DEBUG] Trying to create shared memory segment...")
            self.shm = sysv_ipc.SharedMemory(self.shm_key, sysv_ipc.IPC_CREAT, size=self.shm_size, mode=0o666)
            print("[DEBUG] Shared memory segment created.")
        except sysv_ipc.ExistentialError:
            print("[DEBUG] Shared memory segment exists, attaching...")
            self.shm = sysv_ipc.SharedMemory(self.shm_key)
            print("[DEBUG] Attached to existing shared memory segment.")

    def listener_callback(self, msg):
        print("[DEBUG] Received TimeReference message")
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
        print(f"[DEBUG] Writing to SHM with valid=0, count={st.count}")
        self.shm.write(bytes(st))
        st.valid = 1
        st.count += 1
        print(f"[DEBUG] Writing to SHM with valid=1, count={st.count}")
        self.shm.write(bytes(st))
        self.get_logger().info(f"Published time {sec}.{nsec} to chrony SHM")

def main(args=None):
    print("[DEBUG] Starting main()")
    rclpy.init(args=args)
    node = TimeToSHM()
    print("[DEBUG] Spinning node...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print("[DEBUG] Shutdown complete.")

if __name__ == '__main__':
    main()