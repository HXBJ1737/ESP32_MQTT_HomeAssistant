#ifndef __MAIN_H__
#define __MAIN_H__
#include "Arduino.h"
void Tim1Interrupt();
void Tim2Interrupt();
void max30102_heartrate_init();
void max30102_heartrate_deinit();
void max30102_spo2_init();
float max30102_temp_get();
void max30102_BPM_get();
void max30102_spo2_get();
void show_init0();
void show_init1();
byte key_get();
void spo2_1_set(int time);
void setup_wifi();
void reconnect();
void callback(char *p_topic, byte *p_payload, unsigned int p_length);
void publishkey1_State();
void publishkey2_State();
void publishkey3_State();
void publishkey4_State();
void publishkey5_State();
void publishkey6_State();
void publishkey7_State();
void publishkey8_State();
void setkey1_State();
void setkey2_State();
void setkey3_State();
void setkey4_State();
void setkey5_State();
void setkey6_State();
void setkey7_State();
void setkey8_State();

#endif