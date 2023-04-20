# Percent of charge remaining.  Range is 0 to 100.
# NaN means not valid.
float64 percent_charge_remaining

# Current battery voltage.  Range is 0 to 1000.
# NaN means not valid.
float64 voltage

# The present current draw in amps.  Range is 0 to 1000.
# NaN means not valid.
float64 current_draw

# The current battery temperature in degrees Celcius.
# Range is -75 to 180.  NaN means not valid.
float64 battery_temp

# True if battery hours value is valid.  False if not.
bool is_battery_hours_valid

# Total lifetime number of hours that
# the battery has been in service.
uint32 battery_hours
