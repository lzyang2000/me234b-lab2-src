# Sensor State
#   Active  = 0
#   Standby = 1
#   Off     = 2
bool   sensor_state_valid
uint32 sensor_state

# Zoom Mode
#   Mixed        = 0
#   Analog Only  = 1
#   Digital Only = 2
#   None         = 3
bool   zoom_mode_valid
uint32 zoom_mode

# Zoom level
#   (percentage)
bool    zoom_level_valid
float64 zoom_level

# Effective focal length
#   Unitless (0.0 to 2.0)
bool    focal_length_valid
float64 focal_length

# Horizontal field of view
#   (-pi to pi)
bool    horizontal_field_of_view_valid
float64 horizontal_field_of_view

# Vertical field of view
#   (-pi to pi)
bool    vertical_field_of_view_valid
float64 vertical_field_of_view

# Focus Mode
#   Auto Focus   = 0
#   Manual Focus = 1
bool   focus_mode_valid
uint32 focus_mode

# Is focus on?
bool focus_value_valid
bool focus_value

# White Balance
#   Auto        = 0
#   Daylight    = 1
#   Cloudy      = 2
#   Shade       = 3
#   Tungsten    = 4
#   Fluorescent = 5
#   Flash       = 6
bool   white_balance_valid
uint32 white_balance

# Imaging Mode
#   Color     = 0
#   Greyscale = 1
#   Infrared  = 2
#   Low Light = 3
bool   imaging_mode_valid
uint32 imaging_mode

# Exposure Mode
#   Auto              = 0
#   Manual            = 1
#   Shutter Priority  = 2
#   Aperture Priority = 3
bool   exposure_mode_valid
uint32 exposure_mode

# Metering Mode
#   Auto            = 0
#   Center Weighted = 1
#   Spot            = 2
bool   metering_mode_valid
uint32 metering_mode

# Shutter speed
#   (seconds)
bool    shutter_speed_valid
float64 shutter_speed

# Aperture
#   (f-stop value)
bool    aperture_valid
float64 aperture

# Light Sensitivity Level
#   Auto    = 0
#   ISO100  = 1
#   ISO200  = 2
#   ISO400  = 3
#   ISO800  = 4
#   ISO1600 = 5
#   ISO3200 = 6
bool   light_sensitivity_level_valid
uint32 light_sensitivity_level

# Is image stabilization enabled?
bool image_stabilization_valid
bool image_stabilization
