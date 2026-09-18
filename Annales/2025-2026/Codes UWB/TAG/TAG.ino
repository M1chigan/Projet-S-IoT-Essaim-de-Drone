/*
  Code adapted from James Remington's UWB-Indoor-Localization_Arduino.
  The loop includes UART communication.
*/

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// Leftmost two bytes below will become the "short address"
char tag_addr[] = "cc:db:a7:06:8e:08"; //#4

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// Connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

int received = 0;
int address;
float last_meas = 0;
float last_meas_anchor_1 = 0;
float last_meas_anchor_2 = 0;

void setup() {
  received = 0;

  Serial.begin(115200);
  // Init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  
  // Start the module as a tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_FAST_ACCURACY, false);
}

void loop() {
  if (Serial.available()) {
    received = Serial.read();

    // Measurement retrieval loop
    if (received == 49) { // 49 = "1" in ASCII
      Serial.print("183=");
      Serial.print(last_meas_anchor_1);
      Serial.print(";283=");
      Serial.println(last_meas_anchor_2);

    // Module type recognition loop (1 = anchor, 0 = tag)  
    } else if (received == 51) { // 51 = "3" in ASCII
      Serial.println("0");
    }
    received = 0;
  }
  DW1000Ranging.loop(); // Measurement polling loop
}

void newRange() {
  address = DW1000Ranging.getDistantDevice()->getShortAddress();
  last_meas = DW1000Ranging.getDistantDevice()->getRange();
  
  // Look for addresses 183 (83:01:...) and 283 (83:02:5B:D5:A9:9A:E2:9C)
  if (address == 387) {
    last_meas_anchor_1 = last_meas;
  } else if (address == 643) {
    last_meas_anchor_2 = last_meas;
  }
}