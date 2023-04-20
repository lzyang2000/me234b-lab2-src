# Accepted codes:
# 1 = POWER_ON
# 2 = POWER_OFF
# 3 = POWER_STANDBY
uint8 power_state

# True if valid, false if not.
bool is_rpm_valid

# Revolutions per minute.
uint16 rpm

# Throttle between 0 and 100%.
# NaN if invalid.
float64 throttle

# True if valid, false if not.
bool is_plug_state_valid

# Accepted codes:
# 1 = GLOWPLUG_ON
# 2 = GLOWPLUG_OFF
uint8 plug_state
