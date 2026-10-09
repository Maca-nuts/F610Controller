# C610 motor ID: 1..8
int32 id
# Rotor speed [rps]. Set to 0 for position control.
float32 target_rps
# Output-shaft position [rad], relative to first feedback, unwrapped across turns.
# Converted using gear_ratio (default 36:1); target_rps remains rotor rps.
# Set to 0 for speed control.
# Both zero: zero current. Both nonzero: invalid (zero current).
float32 target_pos
