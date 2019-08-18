#include "lyec_lib.h"
#include <SoftwareSerial.h>
SoftwareSerial lyec_seriall(8,9);
lyec_ardupilot lyec(MAV_TYPE_QUADROTOR,MAV_TYPE_GENERIC,MAV_AUTOPILOT_INVALID,MAV_MODE_PREFLIGHT,0,MAV_STATE_STANDBY);

void setup() {
  Serial.begin(9600);
  lyec_seriall.begin(57600);
  lyec.lyec_hazirla();
}
void loop() {
  lyec.lyec_loop(lyec_seriall);
  if(lyec.mavlink_baglandi){
    Serial.println(lyec.yukselik);
    Serial.println(lyec.x_gyro);
    }

}
7
