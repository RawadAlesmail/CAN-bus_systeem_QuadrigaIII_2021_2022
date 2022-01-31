#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <FastLED.h>
#include <FastLED_NeoPixel.h>


#define SW_Knip_PIN_L_a 9
#define SW_Knip_PIN_R_a 3
#define SW_rem_PIN A0 
#define SW_achterlicht_PIN_a 6
#define SW_alarmlicht_PIN_a 6
#define NUM_LEDS   120
#define LED_PIN     5
#define stuuractuator_1_PIN 12 // deze wordt later gebruikt door de student die actuatoren aanstuurt 
#define stuuractuator_2_PIN 13 // deze wordt later gebruikt door de student die actuatoren aanstuurt
#define SOC_PIN A1
#define Tem_accu_PIN A2

int stuurhoek_value=0; 
int SW_alarmlicht_a=0; 
int SW_achterlicht_a=0;
int SW_knipper_L_a= 0;
int SW_knipper_R_a= 0;
int SW_rem_a=0;
int stuuractuator_1=0; // deze wordt later gebruikt door de student die actuatoren aanstuurt
int stuuractuator_2=0; // deze wordt later gebruikt door de student die actuatoren aanstuurt
int SOC=0;
int Tem_accu=0; 

CRGB leds[NUM_LEDS];

struct can_frame canMsg1; // stuurhoeksensorwaarde data sturen, van mega 1 naar mega 2en3
struct can_frame canMsg2; // knipperlicht_a_L aan/uit, van mega 1 naar 2
struct can_frame canMsg3; // knipperlicht_a_R aan/uit, van mega 1 naar 2
struct can_frame canMsg4; // achterlicht aan/uit, van mega 1 naar 2 
struct can_frame canMsg5; // alarmlicht  aan/uit, van mega 1 naar 2
struct can_frame canMsg6; // display,snelheid, van mega 3 naar 1
struct can_frame canMsg7; // display, RPM, van mega 3 naar 1
struct can_frame canMsg8; // display, SOC, van mega 2 naar 1
struct can_frame canMsg9; // display, Tem accu, van mega 2 naar 1
struct can_frame canMsg10;// display, Tem motorcontroller, van mega 3 naar 1 
struct can_frame canMsg11;// rijrichting, van mega 3 naar 1 

MCP2515 mcp2515(10); // selecteer pin 10

void setup() {
  SPI.begin(); // begin spi communicatie
  Serial.begin(115200);
  
pinMode(SW_Knip_PIN_L_a, INPUT_PULLUP);
pinMode(SW_Knip_PIN_R_a, INPUT_PULLUP); 
pinMode(SW_rem_PIN_a, INPUT_PULLUP); 
pinMode(SW_achterlicht_PIN_a, INPUT_PULLUP);
pinMode(SW_alarmlicht_PIN_a, INPUT_PULLUP);  

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // zet CAN op snelheid 500KBPS
  mcp2515.setNormalMode(); // op normaale mode zetten



canMsg8.can_id  = 0x270;  // SOC
canMsg8.can_dlc = 1;     // can datalengte 1 byte (mag max 8 bytes)
canMsg9.can_id  = 0x105;  // Temperatuur_accu
canMsg9.can_dlc = 1;     // can datalengte 1 byte (mag max 8 bytes)

}

void loop()
{
  
SW_knipper_L_a = digitalRead(SW_Knip_PIN_L_a);
SW_knipper_R_a= digitalRead(SW_Knip_PIN_R_a);
SW_rem_a = digitalRead(SW_rem_PIN_a);
SW_achterlicht_a = digitalRead(SW_achterlicht_PIN_a);
SW_alarmlicht_a = digitalRead(SW_alarmlicht_PIN_a);

SOC=analogRead(A1);/// deze zal later gebruikt worden door de student die BMS maakt
Serial.println(SOC);
Tem_accu=analogRead(A2);/// deze zal later gebruikt worden door de student die BMS maakt
Serial.println(Tem_accu);

////////////////////////////knipperlicht achterrechts///////////
if( SW_knipper_R_a== HIGH) {
for (int i=0; i<NUM_LEDS; i++){
leds[0] = CRGB(255,69,0);
leds[1] = CRGB(255,69,0);
leds[2] = CRGB(255,69,0);
leds[3] = CRGB(255,69,0);
leds[4] = CRGB(255,69,0);
leds[5] = CRGB(255,69,0);
leds[6] = CRGB(255,69,0);
leds[7] = CRGB(255,69,0);
leds[8] = CRGB(255,69,0);
leds[9] = CRGB(255,69,0);
leds[10] = CRGB(255,69,0);
leds[11] = CRGB(255,69,0);
leds[12] = CRGB(255,69,0);
leds[13] = CRGB(255,69,0);
leds[14] = CRGB(255,69,0);
leds[15] = CRGB(255,69,0);
leds[16] = CRGB(255,69,0);
leds[17] = CRGB(255,69,0);
leds[18] = CRGB(255,69,0);
leds[19] = CRGB(255,69,0);
leds[20] = CRGB(255,69,0);   
    FastLED.setBrightness(6*i);
    FastLED.show();
    delay(10);
}}
if(SW_knipper_R_a == LOW) { 
  for (int i=0; i<NUM_LEDS; i++){
    leds[i] = CRGB(0,0,0);
    FastLED.show();}}
 ////////////////////////////knipperlicht achterlinks///////////
if(SW_knipper_L_a== HIGH) {
for (int i=0; i<NUM_LEDS; i++){
leds[100] = CRGB(255,69,0);
leds[101] = CRGB(255,69,0);
leds[102] = CRGB(255,69,0);
leds[103] = CRGB(255,69,0);
leds[104] = CRGB(255,69,0);
leds[105] = CRGB(255,69,0);
leds[106] = CRGB(255,69,0);
leds[107] = CRGB(255,69,0);
leds[108] = CRGB(255,69,0);
leds[109] = CRGB(255,69,0);
leds[110] = CRGB(255,69,0);
leds[111] = CRGB(255,69,0);
leds[112] = CRGB(255,69,0);
leds[113] = CRGB(255,69,0);
leds[114] = CRGB(255,69,0);
leds[115] = CRGB(255,69,0);
leds[116] = CRGB(255,69,0);
leds[117] = CRGB(255,69,0);
leds[118] = CRGB(255,69,0);
leds[119] = CRGB(255,69,0);
leds[120] = CRGB(255,69,0);   
    FastLED.setBrightness(6*i);
    FastLED.show();
    delay(10);
}
}
if(SW_knipper_L_a == LOW) { 
  for (int i=0; i<NUM_LEDS; i++){
    leds[i] = CRGB(0,0,0);
    FastLED.show();}}
//////////////////// Achterlicht //////////////////////

if(SW_Achter_a == HIGH) {        // If button is pressing
for (int i=0; i<NUM_LEDS; i++){
leds[21] = CRGB(255,255,255);
leds[22] = CRGB(255,255,255);
leds[23] = CRGB(255,255,255);
leds[24] = CRGB(255,255,255);
leds[25] = CRGB(255,255,255);
leds[26] = CRGB(255,255,255);
leds[27] = CRGB(255,255,255);
leds[28] = CRGB(255,255,255);
leds[29] = CRGB(255,255,255);
leds[30] = CRGB(255,255,255);
leds[31] = CRGB(255,255,255);
leds[32] = CRGB(255,255,255);
leds[33] = CRGB(255,255,255);
leds[34] = CRGB(255,255,255);
leds[35] = CRGB(255,255,255);
leds[36] = CRGB(255,255,255);
leds[37] = CRGB(255,255,255);
leds[38] = CRGB(255,255,255);
leds[39] = CRGB(255,255,255);
leds[40] = CRGB(255,255,255);


   

leds[80] = CRGB(255,255,255);
leds[81] = CRGB(255,255,255);
leds[82] = CRGB(255,255,255);
leds[83] = CRGB(255,255,255);
leds[84] = CRGB(255,255,255);
leds[85] = CRGB(255,255,255);
leds[86] = CRGB(255,255,255);
leds[87] = CRGB(255,255,255);
leds[88] = CRGB(255,255,255);
leds[89] = CRGB(255,255,255);
leds[90] = CRGB(255,255,255);
leds[91] = CRGB(255,255,255);
leds[92] = CRGB(255,255,255);
leds[93] = CRGB(255,255,255);
leds[94] = CRGB(255,255,255);
leds[95] = CRGB(255,255,255);
leds[96] = CRGB(255,255,255);
leds[97] = CRGB(255,255,255);
leds[98] = CRGB(255,255,255);
leds[99] = CRGB(255,255,255);
   
FastLED.show();
}   
}

if(SW_Achter_a == LOW) { 
  for (int i=0; i<NUM_LEDS; i++){
    leds[i] = CRGB(0,0,0);
    FastLED.show();}}

/////////////////////////////////Remlicht///////////////////////////


if(SW_rem_a == HIGH) {        // If button is pressing
for (int i=0; i<NUM_LEDS; i++){
leds[41] = CRGB(255,255,255);
leds[42] = CRGB(255,255,255);
leds[43] = CRGB(255,255,255);
leds[44] = CRGB(255,255,255);
leds[45] = CRGB(255,255,255);
leds[46] = CRGB(255,255,255);
leds[47] = CRGB(255,255,255);
leds[48] = CRGB(255,255,255);
leds[49] = CRGB(255,255,255);
leds[50] = CRGB(255,255,255);
leds[51] = CRGB(255,255,255);
leds[52] = CRGB(255,255,255);
leds[53] = CRGB(255,255,255);
leds[54] = CRGB(255,255,255);
leds[55] = CRGB(255,255,255);
leds[56] = CRGB(255,255,255);
leds[57] = CRGB(255,255,255);
leds[58] = CRGB(255,255,255);
leds[59] = CRGB(255,255,255);
leds[60] = CRGB(255,255,255);  
leds[61] = CRGB(255,255,255);
leds[62] = CRGB(255,255,255);
leds[63] = CRGB(255,255,255);
leds[64] = CRGB(255,255,255);
leds[65] = CRGB(255,255,255);
leds[66] = CRGB(255,255,255);
leds[67] = CRGB(255,255,255);
leds[68] = CRGB(255,255,255);
leds[69] = CRGB(255,255,255);
leds[70] = CRGB(255,255,255); 
leds[71] = CRGB(255,255,255);
leds[72] = CRGB(255,255,255);
leds[73] = CRGB(255,255,255);
leds[74] = CRGB(255,255,255);
leds[75] = CRGB(255,255,255);
leds[76] = CRGB(255,255,255);
leds[77] = CRGB(255,255,255);
leds[78] = CRGB(255,255,255);
leds[79] = CRGB(255,255,255);
FastLED.show();
}   
}

if(SW_rem_a == LOW) { 
  for (int i=0; i<NUM_LEDS; i++){
    leds[i] = CRGB(0,0,0);
    FastLED.show();}}
//////////////////////////Alarmlicht//////////////////
if(SW_alarmlicht_a == HIGH) {        // If button is pressing
for (int i=0; i<NUM_LEDS; i++){
leds[41] = CRGB(255,255,255);
leds[42] = CRGB(255,255,255);
leds[43] = CRGB(255,255,255);
leds[44] = CRGB(255,255,255);
leds[45] = CRGB(255,255,255);
leds[46] = CRGB(255,255,255);
leds[47] = CRGB(255,255,255);
leds[48] = CRGB(255,255,255);
leds[49] = CRGB(255,255,255);
leds[50] = CRGB(255,255,255);
leds[51] = CRGB(255,255,255);
leds[52] = CRGB(255,255,255);
leds[53] = CRGB(255,255,255);
leds[54] = CRGB(255,255,255);
leds[55] = CRGB(255,255,255);
leds[56] = CRGB(255,255,255);
leds[57] = CRGB(255,255,255);
leds[58] = CRGB(255,255,255);
leds[59] = CRGB(255,255,255);
leds[60] = CRGB(255,255,255);  
leds[61] = CRGB(255,255,255);
leds[62] = CRGB(255,255,255);
leds[63] = CRGB(255,255,255);
leds[64] = CRGB(255,255,255);
leds[65] = CRGB(255,255,255);
leds[66] = CRGB(255,255,255);
leds[67] = CRGB(255,255,255);
leds[68] = CRGB(255,255,255);
leds[69] = CRGB(255,255,255);
leds[70] = CRGB(255,255,255); 
leds[71] = CRGB(255,255,255);
leds[72] = CRGB(255,255,255);
leds[73] = CRGB(255,255,255);
leds[74] = CRGB(255,255,255);
leds[75] = CRGB(255,255,255);
leds[76] = CRGB(255,255,255);
leds[77] = CRGB(255,255,255);
leds[78] = CRGB(255,255,255);
leds[79] = CRGB(255,255,255);  
    FastLED.setBrightness(6*i);
    FastLED.show();
    delay(10);
}   
}

if(SW_alarmlicht_a == LOW) { 
  for (int i=0; i<NUM_LEDS; i++){
    leds[i] = CRGB(0,0,0);
    FastLED.show();}}

 //////////////////////////////////CAN-bus/////////////////////
 
if  (mcp2515.readMessage(&canMsg1)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg1.can_id==0x40)
    {
      int x1= canMsg1.data[0];
    Serial.println(x1);
    }
    }

if  (mcp2515.readMessage(&canMsg2)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg2.can_id==0x340)
    { SW_knipper_L_a==HIGH;}
    }

if  (mcp2515.readMessage(&canMsg3)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg3.can_id==0x350)
    { SW_knipper_R_a==HIGH;}
    }
    
if  (mcp2515.readMessage(&canMsg4)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg4.can_id==0x310)
    { SW_achterlicht_a==HIGH;}
    }

if  (mcp2515.readMessage(&canMsg5)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg5.can_id==0x30)
    { SW_alarmlicht_a==HIGH;}
    }


canMsg8.data[0]=SOC; // updat Value in [0] 
mcp2515.sendMessage(&canMsg8);
delay(200);
canMsg9.data[0]=Tem-accu; // update Value in [0] 
mcp2515.sendMessage(&canMsg9);
delay(200);

}




    
