# header.frame_id is the resolved output frame (see GenerateGrasps.target_frame).
std_msgs/Header header
uint32 num_input_points
float32 inference_time_s
GraspProposal[] grasps
