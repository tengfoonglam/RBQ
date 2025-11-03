std_msgs/Header header

# Contact constants
uint8 CONTACT_UNKNOWN = 0
uint8 CONTACT_MADE = 1
uint8 CONTACT_LOST = 2

# Arrays for 4 legs (FL, FR, RL, RR)
geometry_msgs/Point[] foot_position_rt_body    # Foot position relative to body frame (4 elements)
geometry_msgs/Point[] foot_velocity_rt_body    # Foot velocity relative to body frame (4 elements)
uint8[] contact                                 # Contact state for each foot (4 elements)
