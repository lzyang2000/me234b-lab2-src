# Accepted codes:
# 1 = POWER_ON
# 2 = POWER_OFF
# 3 = POWER_STANDBY
uint8 power_state

# Is RPM valid?
bool is_rpm_valid

# Revolutions per minute.
uint16 rpm

# Throttle between 0 and 100%.  NaN if invalid.
float64 throttle
