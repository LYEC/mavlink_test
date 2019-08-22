/*
 *       II         II     II     IIIIIII      IIIIIIII
 *       II          II   II      II          II
 *       II           IIII        IIIIIII     II
 *       II            II         II          II
 *       IIIIII        II         IIIIIII      IIIIIIII       
 */
 
#ifndef lyec_lib_h
#define lyec_lib_h

#include <Arduino.h>
#include "mavlink.h"

class lyec_ardupilot {
  public:
   lyec_ardupilot(int l_tip,uint8_t l_sistem_tipi , uint8_t l_otopilot_tipi , uint8_t l_sistem_modu, uint32_t l_ozel_mod , uint8_t l_sistem_durum);
   void lyec_loop(Stream& lyec_serial);
   void lyec_hazirla();
   void mavlink_verial(Stream& lyec_serial);
   //void 
   void veri_akis_paketi_al(int sysid,int compid ,mavlink_message_t mesaj,int tarsys,int tarcomp,int akis,int mesaj_hiz,int baslat,Stream& lyec_serial);
   void mavlink_cmd_yolla(uint16_t komut, int sysid,int compid,mavlink_message_t mesaj,int tarsys,int tarcomp, float p1, float p2, float p3, float p4, float p5, float p6, float p7,Stream& lyec_serial);
   void mavlink_rc_yolla(int sysid,int compid,mavlink_message_t mesaj,int tarsys,int tarcomp,float roll, float pitch, float yaw, float thrust, float modeCh,Stream& lyec_serial);
   //değişkenler
   uint8_t mavlink_baglandi;
   uint8_t ap_tip;
   uint8_t ap_otopilot;
   uint8_t ap_taban_mod;
   uint8_t ap_sistem_durumu;
   uint8_t ap_mavlik_versiyon;
   uint32_t ap_ozel_mod;
   uint16_t pil_voltaj;
   int16_t  pil_amper;
   uint16_t bozuk_paket_sayisi;
   uint8_t gps_durum; // 0 => GPS YOK , 1 => kalibre yok , 2 => 2D kalibre , 3 => 3D kalibre
   int32_t gps_enlem;
   int32_t gps_boylam;
   int32_t gps_yukselik; // mm
   float ucus_hizi;  // m/s
   float yere_gore_hiz; // m/s
   uint16_t gps_epv;
   uint16_t gps_eph;
   uint16_t gps_hiz;
   float yukselik;
   float yukselme_orani;
  // HAM VERİ
   int16_t x_hizlanma;  
   int16_t y_hizlanma;
   int16_t z_hizlanma;
   int16_t x_gyro; // ivme sensörü 
   int16_t y_gyro;
   int16_t z_gyro;
   int16_t x_maynetik;
   int16_t y_manyetik;
   int16_t z_manyetik;
    private:
   int tip;
   uint8_t sistem_tipi;
   uint8_t otopilot_tipi;
   uint8_t sistem_modu;
   uint32_t ozel_mod;
   uint8_t sistem_durum;
   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
   uint16_t len;
};

#endif  

