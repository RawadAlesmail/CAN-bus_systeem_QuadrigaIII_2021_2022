#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <FastLED.h>
#include <FastLED_NeoPixel.h>

#define stuurhoeksensor A0 
#define SW_Knip_PIN_L_v 3
#define SW_Knip_PIN_R_v 6
#define SW_kop_PIN_v 4
#define SW_toeter_PIN_v 5
#define SW_ruitenwisser_PIN_v 2
#define SW_alarmlicht_PIN_a 17
#define SW_achterlicht_PIN_a 18
#define LED_PIN     7
#define toeter_PIN= 19
#define ruitenwisser_PIN 20
#define NUM_LEDS   120

int stuurhoek_value=0; 
int SW_knipper_L_v = 0;
int SW_knipper_R_v = 0;
int SW_kop_v=0;
int SW_toeter=0;
int SW_ruitenwisser=0;
int SW_alarmlicht_a=0; 
int SW_achterlicht_a=0;
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
struct can_frame canMsg12;
struct can_frame canMsg13;
struct can_frame canMsg14;
struct can_frame canMsg15;
struct can_frame canMsg16; 
struct can_frame canMsg17;
struct can_frame canMsg18;
struct can_frame canMsg19;
struct can_frame canMsg20;

MCP2515 mcp2515(10); // selecteer pin 10

void setup() {
SPI.begin(); // begin spi communicatie
Serial.begin(115200);

pinMode(SW_Knip_PIN_L_v, INPUT_PULLUP);
pinMode(SW_Knip_PIN_R_v, INPUT_PULLUP); 
pinMode(SW_kop_PIN_v, INPUT_PULLUP);
pinMode(SW_toeter_PIN_v, INPUT_PULLUP);
pinMode(SW_ruitenwisser_PIN_v, INPUT_PULLUP); 
pinMode(SW_alarmlicht_PIN_a, INPUT_PULLUP); 
pinMode(SW_achterlicht_PIN_a, INPUT_PULLUP); 
pinMode(toeter_PIN, OUTPUT);
pinMode(ruitenwisser_PIN, OUTPUT);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

 
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ); // zet CAN op snelheid 500KBPS
  mcp2515.setNormalMode();
  
canMsg1.can_id  = 0x40;  // stuursensorwaarde sturen van mega 1 naar mega 2 en 3 
canMsg1.can_dlc = 3;     // can datalengte 1 byte (mag max 8 bytes)
canMsg2.can_id  = 0x340;  // knipperlicht-a-L, sturen van mega 1 naar mega 2 
canMsg2.can_dlc = 1;     // can datalengte 1 byte (mag max 8 bytes)
canMsg3.can_id  = 0x350;  // knippperlicht-a-R, sturen van mega 1 naar mega 2 
canMsg3.can_dlc = 1;     // can datalengte 1 byte (mag max 8 bytes)
canMsg4.can_id  = 0x310;  // achterlicht, sturen van mega 1 naar mega 2 
canMsg4.can_dlc = 1;     // can datalengte 1 byte (mag max 8 bytes)
canMsg5.can_id  = 0x30;  // alarmlicht, sturen van mega 1 naar mega 2
canMsg5.can_dlc = 1;     // can datalengte 1 byte (mag max 8 bytes) 
}

void loop() {
stuurhoek_value=analogRead(A0);
Serial.println(stuurhoek_value);

SW_knipper_L_v = digitalRead(SW_Knip_PIN_L_v);
SW_knipper_R_v = digitalRead(SW_Knip_PIN_R_v);
SW_kop_v = digitalRead(SW_kop_PIN_v);
SW_toeter_v = digitalRead(SW_toeter_PIN_v);
SW_ruitenwisser_v = digitalRead(SW_ruitenwisser_PIN_v);
SW_alarmlicht_a = digitalRead(SW_alarmlicht_PIN_a);
SW_achterlicht_a = digitalRead(SW_achterlicht_PIN_a);
if( SW_toter== HIGH) {
  digitalWrite(toeter_PIN, HIGH);}
else {
  digitalWrite(toeter_PIN, LOW);}
 if( SW_ruitenwisser== HIGH) {
  digitalWrite(ruitenwisser_PIN, HIGH);}
else {
  digitalWrite(ruitenwisser_PIN, LOW);}
  
  
}
////////////////////////////knipperlicht voorrechts///////////
if( SW_knipper_R_v== HIGH) {
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
if(SW_knipper_R_v == LOW) { 
  for (int i=0; i<NUM_LEDS; i++){
    leds[i] = CRGB(0,0,0);
    FastLED.show();}}
 ////////////////////////////knipperlicht voorlinks///////////
if(SW_knipper_L_v == HIGH) {
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
if(SW_knipper_L_v == LOW) { 
  for (int i=0; i<NUM_LEDS; i++){
    leds[i] = CRGB(0,0,0);
    FastLED.show();}}
//////////////////// Koplampen //////////////////////

if(SW_kop_v == HIGH) {        // If button is pressing
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
  delay(10);
FastLED.show();

}   
}

if(SW_kop_v == LOW) { 
  for (int i=0; i<NUM_LEDS; i++){
    leds[i] = CRGB(0,0,0);
    FastLED.show();}}

/////////////////////////////////CAN_BUS//////////////////////
  if  (mcp2515.readMessage(&canMsg6)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg6.can_id==0x065)
    {
      int x6= canMsg6.data[0];
      Serial.print("snelheid:   ");
      Serial.println(x6);}}

  if  (mcp2515.readMessage(&canMsg7)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg7.can_id==0x065)
    {
      int x7= canMsg7.data[0];
      Serial.print("RPM:   ");
      Serial.println(x7);}}

  if  (mcp2515.readMessage(&canMsg8)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg8.can_id==0x065)
    {
      int x8= canMsg8.data[0];
      Serial.print("SOC:   ");
      Serial.println(x8);}}
      
  if  (mcp2515.readMessage(&canMsg9)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg9.can_id==0x065)
    {
      int x9= canMsg9.data[0];
      Serial.print("Tem_accu:   ");
      Serial.println(x9);}}

  if  (mcp2515.readMessage(&canMsg10)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg10.can_id==0x065)
    {
      int x10= canMsg10.data[0];
      Serial.print("Tem_MC:   ");
      Serial.println(x10);}}

  if  (mcp2515.readMessage(&canMsg11)== MCP2515::ERROR_OK)  // om data te ontvangen
  {
    if(canMsg11.can_id==0x065)
    {
      int x11= canMsg11.data[0];
      Serial.print("rijrichting:   ");
      Serial.println(x11);}}


// if (digitalRead(BUTTON_PIN) == HIGH) {
//    digitalWrite(LED_PIN, HIGH);
//  }
//  else {
//    digitalWrite(LED_PIN, LOW);
//  }
  
// stuurhoek_value= analogRead(A0);
// stuurhoek_value= map(stuurhoek_value,0,1023,0,225);
// Serial.print("stuurhoek_value:  ");
// Serial.println(stuurhoek_value);


 
canMsg1.data[0]=stuurhoek_value; // update Value in [0] 
mcp2515.sendMessage(&canMsg1);
delay(200);


if (SW_knipper_L_v==HIGH)
{canMsg2.data[0]=SW_knipper_L_v; // update Value in [0] 
mcp2515.sendMessage(&canMsg2);
delay(200);}

if (SW_knipper_R_v==HIGH)
{canMsg3.data[0]=SW_knipper_R_v; // update Value in [0] 
mcp2515.sendMessage(&canMsg3);
delay(200);}

if (SW_achterlicht_a==HIGH)
{canMsg4.data[0]=SW_achterlicht_a; // update Value in [0] 
mcp2515.sendMessage(&canMsg4);
delay(200);}

if (SW_alarmlicht_a==HIGH)
{canMsg5.data[0]=SW_alarmlicht_a; // update Value in [0] 
mcp2515.sendMessage(&canMsg5);
delay(200);}

}
