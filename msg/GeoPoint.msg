# WGS84 latitude, longitude, and MSL altitude
float64 lat
float64 lon
float64 alt

# Relative altitude (custom definable, but generally set w.r.t. some home point)
float64 rel_alt

# Flag field for which altitudes are valid
bool using_alt
bool using_rel_alt
