#ifndef RADIORECEIVER_H_INCLUDED
#define RADIORECEIVER_H_INCLUDED

#include <SPI.h>
#include "RF24.h"

#define PIN_SPI_CE      9
#define PIN_SPI_CSN     10

class RadioReceiver {

private:
    RF24 m_radio= RF24(PIN_SPI_CE, PIN_SPI_CSN);      // Object to control NRF24L01
    const byte m_addresses[6];              //set commucation address, same to remote controller
    int m_nrfDataRead[8];                             //define an array to save data from remote controller
    bool m_flagListening{false};                      // false there is no NRF CHIP
    bool m_flagReceivingData{false};                  // true if data are received

public:

    // Constructor with initialization of the cons member 
    RadioReceiver(): m_addresses("Free1"){ }
    
    // is the NRF Chip Found?
    bool isListening();

    // is the car receiving data?
    bool isReceiving();

    // setup of the receiver
    void setupReceiver();

    // receive data from remote controller
    void receiveData();

    // obtain Data
    int* getData();

    // print the Data on the Serial monitor
    void printData();


};


#endif // RADIORECEIVER_H_INCLUDED
