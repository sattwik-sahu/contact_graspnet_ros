# Tool/EE pose in the frame of GraspProposalArray.header.
# With the default tool transform this is the Contact-GraspNet gripper-base
# pose: +Z is the approach direction and +X is the finger-closing direction.
# The node's tcp_offset/tcp_rpy parameters map it to a downstream tool frame.
geometry_msgs/Pose pose
float32 score
float32 width
geometry_msgs/Point contact_point
int32 segment_id
