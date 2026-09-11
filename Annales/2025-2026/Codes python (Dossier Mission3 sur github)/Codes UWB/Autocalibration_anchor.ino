#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// leftmost two bytes below will become the "short address"
char anchor_addr[] = "83:01:5B:D5:A9:9A:E2:9C"; //#4

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16602;

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin
bool start_calibration = false;
int received;

float this_anchor_target_distance = 1; //measured distance to anchor in m

uint16_t this_anchor_Adelay = 16600; //starting value
uint16_t Adelay_delta = 100; //initial binary search step size

void setup()
{ 
  received = 0;

  Serial.begin(115200);
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  DW1000Ranging.attachNewRange(newRange);

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

  //start the module as an anchor, do not assign random short address
  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_FAST_ACCURACY, false);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_LOWPOWER);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_ACCURACY);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_ACCURACY);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop()
{
  if (Serial.available()) {
    received = Serial.read();
    if(received == 50){// 50 = "2" en ASCII
      start_calibration = true;
      Adelay_delta = 100;
    }
    received = 0;
  }
  DW1000Ranging.loop();
}


void newRange()
{
  if(start_calibration){
    static float last_delta = 0.0;
    float dist = DW1000Ranging.getDistantDevice()->getRange();

    if (Adelay_delta < 3) {
      Serial.println(this_anchor_Adelay);
      DW1000.setAntennaDelay(this_anchor_Adelay);
      start_calibration = false;
      return;
    }

    float this_delta = dist - this_anchor_target_distance;  //error in measured distance

    if ( this_delta * last_delta < 0.0) Adelay_delta = Adelay_delta / 2; //sign changed, reduce step size
      last_delta = this_delta;
    
    if (this_delta > 0.0 ) this_anchor_Adelay += Adelay_delta; //new trial Adelay
    else this_anchor_Adelay -= Adelay_delta;

    DW1000.setAntennaDelay(this_anchor_Adelay);
  }
}