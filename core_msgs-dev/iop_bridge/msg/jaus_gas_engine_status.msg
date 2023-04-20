# Percent fuel remaining.  Range is 0 to 100.
# NaN means not valid.
float64 percent_remaining_fuel

# Oil pressure above 100% represents
# an overpressure condition.  Range is 0 to 200.
# NaN means not valid.
float64 percent_oil_pressure

# Percent of nominal oil level.
# Range is 0 to 100.
# NaN means not valid.
float64 percent_oil_level

# Oil temperature in degrees Celcius.
# Range is -75 to 180.
# NaN means not valid.
float64 oil_temp

# Engine temperature in degrees Celcius.
# Range is -75 to 180.
# NaN means not valid.
float64 engine_temp

# True if engine_hours is a valid measurement.
# False if not.
bool is_engine_hours_valid

# The lifetime number of hours this engine
# has been in service.
uint32 engine_hours

# True if tachometer is a valid measurement.
# False if not.
bool is_tachometer_valid

# Revolutions per minute of engine.
uint16 tachometer

# Coolant level in liters.
# Range is 0 to 256.
# NaN means not valid.
float64 coolant_level

# Coolant temperature in degrees Celcius.
# Range is -75 to 180.
# NaN means not valid.
float64 coolant_temp

# Air intake pressure in bar.
# Range is 0 to 512.
# NaN means not valid.
float64 air_intake_pressure

# Air intake temperature in degrees Celcius.
# Range is -75 to 180.
# NaN means not valid.
float64 air_intake_temp

# Engine exhaust temperature in degrees Celcius.
# Range is -75 to 180.
# NaN means not valid.
float64 exhaust_temp
