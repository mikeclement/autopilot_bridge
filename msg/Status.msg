uint8 MODE_RALLY=0
uint8 MODE_MANUAL=1
uint8 MODE_FBWA=2
uint8 MODE_GUIDED=3
uint8 MODE_AUTO=4
uint8 MODE_FBWB=5
uint8 MODE_CIRCLE=6
uint8 MODE_LOITER=7
uint8 MODE_INITIALIZING=8
uint8 MODE_UNKNOWN=15

uint8 FENCE_NOT_BREACHED=0
uint8 FENCE_BREACHED=1
uint8 FENCE_DISABLED=2

Header  header
uint8   mode
bool    armed
bool    ahrs_ok
int32   alt_rel
bool    as_ok
float32 as_read
uint8   fence_status
bool    gps_ok
uint8   gps_sats
uint16  gps_eph
bool    ins_ok
bool    mag_ok
uint16  mis_cur
bool    pwr_ok
int16   pwr_batt_rem
int16   pwr_batt_vcc
int16   pwr_batt_cur

