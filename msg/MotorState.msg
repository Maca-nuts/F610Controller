# ID and feedback used by the controller; published at approximately 100 Hz.
int32 id
# Output-shaft position [rad], relative to first feedback.
float64 position_rad
# Unwrapped rotor position [rad].
float64 rotor_position_rad
# Rotor speed [rps].
float64 velocity_rps
float64 rotor_rpm
float64 commanded_current_ma
float64 velocity_integral_ma
