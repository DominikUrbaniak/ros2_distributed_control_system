import rclpy.qos as qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Define QoS profiles

qos_profile_R10 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_R5 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=5,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_R20 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=20,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_R10 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_R1 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1,                                     # Keep one message in history 1
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_R1_deadline = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1,                                     # Keep one message in history 1
    durability=QoSDurabilityPolicy.VOLATILE,
    deadline=qos.Duration(seconds=0, nanoseconds=4000000),
    #lifespan=qos.Duration(seconds=5, nanoseconds=0),
)
qos_profile_B10 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10,                                      # Keep 10 messages in history 10
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_B1 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1,                                      # Keep one message in history 1
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_udp = qos.QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_tcp = qos.QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_BV = qos.QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_RT = qos.QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_RV = qos.QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_BT = qos.QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_RV10 = qos.QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_RT1 = qos.QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
qos_profile_RV1 = qos.QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.VOLATILE,
)
qos_profile_RT10 = qos.QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
