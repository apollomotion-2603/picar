std_msgs/Header header

# Lateral offset from lane center (n in NMPC state vector) [m]
float64 e_y

# Heading error relative to lane tangent (alpha in NMPC state vector) [rad]
float64 e_psi

# Curvature of centerline at s=0 (vehicle position) [1/m]
float64 kappa

# Cubic polynomial centerline in BEV arc-length frame:
#   x_c(s) = coeff_a*s^3 + coeff_b*s^2 + coeff_c*s + coeff_d
# s = arc-length from vehicle [m], x_c = lateral position [m]
float64 coeff_a
float64 coeff_b
float64 coeff_c
float64 coeff_d

# Maximum valid arc-length (camera lookahead limit) [m]
float64 s_max

# Whether lane was successfully detected
bool lane_detected
