/*
  Ce code est largement repris du github de James Remington https://github.com/jremington/UWB-Indoor-Localization_Arduino
  La seule contribution apportée est la boucle loop qui intègre la communication UART
*/

// la librairie doit être récupérée dans le github fourni au dessus
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// leftmost two bytes below will become the "short address"
char tag_addr[] = "85:00:5B:D5:A9:9A:E2:9C"; //#4

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin


int received = 0;
int address;
float last_meas = 0;
float last_meas_anchor_1 = 0;
float last_meas_anchor_2 = 0;

void setup()
{
  received = 0;

  Serial.begin(115200);
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  DW1000Ranging.attachNewRange(newRange);
  

  //start the module as an anchor, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_FAST_ACCURACY, false);
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

    // boucle récupération des mesures
    if(received == 49){// 49 = "1" en ASCII
      Serial.print("183=");
      Serial.print(last_meas_anchor_1);
      Serial.print(";283=");
      Serial.println(last_meas_anchor_2);

    // boucle reconnaissance type de module (1 = anchor, 0 = tag)  
    }else if(received == 51){// 51 = "3" en ASCII
      Serial.println("0");
    }
    received = 0;
  }
  DW1000Ranging.loop();// boucle de prise de mesure
}

void newRange()
{
  address = DW1000Ranging.getDistantDevice()->getShortAddress();
  last_meas = DW1000Ranging.getDistantDevice()->getRange();
  //cherhe les addresses 183 (83:01:...) et 283 (83:02:5B:D5:A9:9A:E2:9C)
  if(address == 387){
    last_meas_anchor_1 = last_meas;
  }else if(address == 643){
    last_meas_anchor_2 = last_meas;
  }
}