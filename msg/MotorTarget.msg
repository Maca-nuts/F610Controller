# C610 motor ID: 1..8
int32 id
# Rotor speed [rps]. Set to 0 for position control.
float64 target_rps
# Output-shaft position [rad], relative to first feedback, unwrapped across turns.
# Converted using gear_ratio (default 36:1); target_rps remains rotor rps.
# In AUTO mode: zero selects speed control; both zero means zero current.
# POSITION mode: zero commands the origin. Unused targets must be zero.
float64 target_pos
# AUTO preserves the legacy nonzero-field selection. POSITION accepts zero.
uint8 AUTO=0
uint8 STOP=1
uint8 VELOCITY=2
uint8 POSITION=3
uint8 mode
