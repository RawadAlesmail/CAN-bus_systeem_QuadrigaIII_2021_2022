#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>

#define motorcontroller_PIN 10 // deze zal gebruikt worden door de student die de motorcontroller maakt 
#define motorcontroller_PIN A1 // deze zal gebruikt worden door de student die de motorcontroller maakt 

int MC_snelheid=0; // deze zal gebruikt worden door de student die de motorcontroller maakt 
int MC_RPM=0; // deze zal gebruikt worden door de student die de motorcontroller maakt 
int MC_Tem_MC=0; // deze zal gebruikt worden door de student die de motorcontroller maakt 

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

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS,MCP_8MHZ); // zet CAN op snelheid 500KBPS
  mcp2515.setNormalMode(); // op normaale mode zetten

canMsg6.can_id  = 0x70;  // snelheid 
canMsg6.can_dlc = 2;     // can datalengte 1 byte (mag max 8 bytes) 
canMsg7.can_id  = 0x90;  // RPM
canMsg7.can_dlc = 2;     // can datalengte 1 byte (mag max 8 bytes)
canMsg10.can_id  = 0x220;  // Tem_MC
canMsg10.can_dlc = 1;     // can datalengte 1 byte (mag max 8 bytes)
}

void loop() 
{

 
/////////////////// CAN-bus ////////////////////////////
 
if  (mcp2515.readMessage(&canMsg1)== MCP2515::ERROR_OK)  // om data te ontvangen 
{ 
  if(canMsg1.can_id==0x40)
  {
    int x1= canMsg1.data[0];
    Serial.println("stuurhoekwaarde:    ");
    Serial.println(x1);}
}

canMsg6.data[0]=MC_snelheid; // update pot Value in [0] 
mcp2515.sendMessage(&canMsg6);
delay(200);

canMsg7.data[0]=MC_RPM; // update pot Value in [0] 
mcp2515.sendMessage(&canMsg7);
delay(200);

canMsg10.data[0]=MC_Tem_MC; // update pot Value in [0] 
mcp2515.sendMessage(&canMsg10);
delay(200);





}
 
