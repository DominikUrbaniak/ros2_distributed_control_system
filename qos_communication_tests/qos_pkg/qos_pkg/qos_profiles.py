import rclpy.qos as qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Define QoS profiles
qos_profile_RV10D1 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
    deadline=qos.Duration(seconds=0, nanoseconds=1000000),
)
qos_profile_RV10D2 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
    deadline=qos.Duration(seconds=0, nanoseconds=2000000),
)
qos_profile_RV10D5 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
    deadline=qos.Duration(seconds=0, nanoseconds=5000000),
)
qos_profile_RV10D10 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
    deadline=qos.Duration(seconds=0, nanoseconds=10000000),
)
qos_profile_RV10 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_R10 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
)
qos_profile_RV1 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1,                                     # Keep one message in history 1
    durability=QoSDurabilityPolicy.VOLATILE,
)

qos_profile_RV100 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=100,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_BV1 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1,                                      # Keep one message in history 1
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_BV10 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_BV100 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=100,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_RT1 = qos.QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_RT10 = qos.QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_RT100 = qos.QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=100,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_BT1 = qos.QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_BT10 = qos.QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_BT100 = qos.QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
