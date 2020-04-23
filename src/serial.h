/*
 *  ESP8266         32U4RFM95
 *  ------------------------------
 *  RXD2 [13]  <--  TXD1 [1]
 * 
 */

#define SERIAL_PIN 13

bool init_serial();
void poll_packet();