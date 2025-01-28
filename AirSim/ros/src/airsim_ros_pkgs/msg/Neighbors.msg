# Neighbors.msg
# # float64[] self_drone_pose: pose of self drone x, y, z, theta

Header header
int32 drone_id
int32 decision_making_id
float64[] self_drone_pose
int32[] in_neighbor_ids
int32[] out_neighbor_ids
int32[] robots_range_bearing_ids
float64[] robots_range_bearing_values
int32[] graph_nodes
int32[] graph_edges
