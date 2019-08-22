/*
 *       II         II     II     IIIIIII      IIIIIIII
 *       II          II   II      II          II
 *       II           IIII        IIIIIII     II
 *       II            II         II          II
 *       IIIIII        II         IIIIIII      IIIIIIII          
 */

#include "lyec_lib.h"

unsigned long lyec_millis_p = 0;
unsigned long lyec_millis_n = 500;
const int beklencek_heartbeats = 60; // beklencek kalp atışı sayısı
int b_heartbeats = beklencek_heartbeats;
int baglanti_test[10];

lyec_ardupilot::lyec_ardupilot(int l_tip,uint8_t l_sistem_tipi , uint8_t l_otopilot_tipi , uint8_t l_sistem_modu, uint32_t l_ozel_mod , uint8_t l_sistem_durum){
  tip=l_tip;
  sistem_tipi=l_sistem_tipi;
  otopilot_tipi=l_otopilot_tipi;
  sistem_modu=l_sistem_modu;
  ozel_mod=l_ozel_mod;
  sistem_durum=l_sistem_durum;
}
+
void lyec_ardupilot::lyec_loop(Stream& lyec_serial){
mavlink_message_t mesaj;  
//mesajı paketle
mavlink_msg_heartbeat_pack(1,0, &mesaj, tip, otopilot_tipi, sistem_modu, ozel_mod, sistem_durum);
// mesajı , iletici tampona kopyala
uint16_t len = mavlink_msg_to_send_buffer(buf, &mesaj);
unsigned long lyec_millis_c = millis();
if (lyec_millis_c - lyec_millis_p >= lyec_millis_n) {
  lyec_millis_p = lyec_millis_c;
  lyec_serial.write(buf,len);
  b_heartbeats++;
  if(b_heartbeats>=beklencek_heartbeats){
      mavlink_message_t t_mesaj;
      veri_akis_paketi_al(200,190,t_mesaj,1,0,MAV_DATA_STREAM_EXTRA2,30,1,lyec_serial);
      delay(10);
      veri_akis_paketi_al(200,190,t_mesaj,1,0,MAV_DATA_STREAM_RAW_SENSORS,10,1,lyec_serial);
      delay(10);
      veri_akis_paketi_al(200,190,t_mesaj,1,0,MAV_DATA_STREAM_EXTENDED_STATUS,10,1,lyec_serial);
      b_heartbeats=0;
      }
}
mavlink_verial(lyec_serial);
}

void lyec_ardupilot::lyec_hazirla(){
digitalWrite(16,LOW);
mavlink_baglandi = 0;
delay(3000);
//digitalWrite(16,HIGH);
}

void lyec_ardupilot::mavlink_verial(Stream& lyec_serial){
  mavlink_message_t msg;
  mavlink_status_t status;
  while(lyec_serial.available()) 
  { 
    uint8_t c = lyec_serial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    {
      switch(msg.msgid)
      {
          case MAVLINK_MSG_ID_HEARTBEAT:  
          ap_taban_mod = (mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7;
          ap_ozel_mod = mavlink_msg_heartbeat_get_custom_mode(&msg);
          //Serial.println(mavlink_msg_heartbeat_get_system_status(&msg));
          break;
          case MAVLINK_MSG_ID_SYS_STATUS:
          if(!mavlink_baglandi){mavlink_baglandi=1;digitalWrite(16,HIGH);}
          mavlink_sys_status_t status_paket;
          mavlink_msg_sys_status_decode(&msg, &status_paket);
          pil_voltaj = status_paket.voltage_battery;
          pil_amper = status_paket.current_battery;
          bozuk_paket_sayisi = status_paket.drop_rate_comm;
          break;
          case MAVLINK_MSG_ID_VFR_HUD:
          if(!mavlink_baglandi){mavlink_baglandi=1;digitalWrite(16,HIGH);}
          mavlink_vfr_hud_t vfr_paket;
          mavlink_msg_vfr_hud_decode(&msg,&vfr_paket);
          yukselik = vfr_paket.alt;
          yukselme_orani = vfr_paket.climb;
          break;
          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          if(!mavlink_baglandi){mavlink_baglandi=1;digitalWrite(16,HIGH);}
          mavlink_global_position_int_t global_pos_paket;
          mavlink_msg_global_position_int_decode(&msg,&global_pos_paket);
          break;
          case MAVLINK_MSG_ID_RAW_IMU:
          if(!mavlink_baglandi){mavlink_baglandi=1;digitalWrite(16,HIGH);}
          mavlink_raw_imu_t paket_raw;
          mavlink_msg_raw_imu_decode(&msg, &paket_raw);
          x_hizlanma = paket_raw.xacc;
          y_hizlanma = paket_raw.yacc;
          z_hizlanma = paket_raw.zacc;
          x_gyro = paket_raw.xgyro;
          y_gyro = paket_raw.ygyro;
          z_gyro = paket_raw.zgyro;
          x_maynetik = paket_raw.xmag;
          y_manyetik = paket_raw.ymag;
          z_manyetik = paket_raw.zmag;
          break;
          case MAVLINK_MSG_ID_GPS_RAW_INT:
          if(!mavlink_baglandi){mavlink_baglandi=1;digitalWrite(16,HIGH);}
          mavlink_gps_raw_int_t gps_paket;
          mavlink_msg_gps_raw_int_decode(&msg,&gps_paket);
          gps_durum = gps_paket.fix_type;
          if(gps_durum==3){
            gps_enlem = gps_paket.lat;
            gps_boylam = gps_paket.lon;
            gps_yukselik = gps_paket.alt;
          }
          break;
          default:
          //if(!mavlink_baglandi){mavlink_baglandi=1;digitalWrite(16,HIGH);}
          break;
      }
}
  }
}

void lyec_ardupilot::veri_akis_paketi_al(int sysid,int compid ,mavlink_message_t mesaj,int tarsys,int tarcomp,int akis,int mesaj_hiz,int baslat,Stream& lyec_serial){
  mavlink_msg_request_data_stream_pack(sysid, compid, &mesaj, tarsys, tarcomp, akis, mesaj_hiz, baslat);
  len = mavlink_msg_to_send_buffer(buf, &mesaj);
  lyec_serial.write(buf,len);
}

void lyec_ardupilot::mavlink_cmd_yolla(uint16_t komut, int sysid,int compid,mavlink_message_t mesaj,int tarsys,int tarcomp,float p1, float p2, float p3, float p4, float p5, float p6, float p7,Stream& lyec_serial){
  mavlink_msg_command_long_pack(sysid, compid, &mesaj, tarsys, tarcomp, komut, 0, p1, p2, p3, p4, p5, p6, p7);
  len = mavlink_msg_to_send_buffer(buf, &mesaj);
  lyec_serial.write(buf,len);
}

void lyec_ardupilot::mavlink_rc_yolla(int sysid,int compid,mavlink_message_t mesaj,int tarsys,int tarcomp,float roll, float pitch, float yaw, float thrust, float modeCh,Stream& lyec_serial){
   mavlink_msg_rc_channels_override_pack(sysid, compid, &mesaj, tarsys, tarcomp, roll, pitch, thrust, yaw, modeCh, -1, -1, -1);
   len = mavlink_msg_to_send_buffer(buf, &mesaj);
   lyec_serial.write(buf,len);
}
