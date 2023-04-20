# Fluid level in liters.  Range is 0-256.
# NaN means not valid.
float64 fluid_level

# Percentage of fluid pressure.  Range is 0-200.
# Values over 100 represent overpressure.
# NaN means not valid.
float64 percent_fluid_pressure

# Fluid temperature in degrees Celcius.
# Range is -75 to 180.  NaN means not valid.
float64 fluid_temp

# True if total_hours value is valid,
# false if not.
bool is_total_hours_valid

# Total lifetime hours of service by this hydraulic
# system.
uint32 total_hours
