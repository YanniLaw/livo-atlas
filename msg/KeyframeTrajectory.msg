uint8 READY=0
uint8 RESOURCE_LIMITED=1
uint8 ERROR=2

std_msgs/Header header
string producer_id
uint64 revision
uint64 pose_version
uint64 frame_count
uint64 optimized_frame_count
uint8 status
string error
TrajectoryKeyframe[] keyframes
