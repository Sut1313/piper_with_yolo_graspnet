std_msgs/Header header

# Normalized plane in header.frame_id: a*x + b*y + c*z + d = 0.
float64[4] coefficients

# Shortest distance from the camera origin to the plane.
float64 perpendicular_distance_m

# Positive intersection depth on optical +Z; NaN when it is undefined.
float64 optical_axis_distance_m

uint32 input_point_count
uint32 filtered_point_count
uint32 inlier_count
float64 inlier_ratio
float64 rms_error_m
float64 normal_error_deg

bool valid
string status
