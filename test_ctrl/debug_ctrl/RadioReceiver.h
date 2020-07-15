/*
This class is used to decode the information gathered by the radio and it is converted into useful data to be used for planning and control tasks
*/

#ifndef RADIORECEIVER_H_INCLUDED
#define RADIORECEIVER_H_INCLUDED

#include <SPI.h>
#include "RF24.h"

#define PIN_SPI_CE      9
#define PIN_SPI_CSN     10



class RadioReceiver {

private:
    RF24 m_radio= RF24(PIN_SPI_CE, PIN_SPI_CSN);  	    			// Object to control NRF24L01
    const byte m_addresses[6];             				 	//set commucation address, same to remote controller
    int m_nrfDataRead[8];                             				//define an array to save data from remote controller
    bool m_flagListening{false};                      				// false there is no NRF CHIP on the car
    bool m_flagReceivingData{false};                  				// true if data are from the remote are received (i.e. no loss of communication or remote is not switched off)

public:

 
    RadioReceiver(): m_addresses("Free1"){ }
    /*
    default constructor 
    */
    


    bool isListening();
    /*
    state if the NRF chip has been found and set the corresponding flag 
    */

    
    bool isReceiving();
    /*
    state if the car is receiving data from the remote  
    */


    void setupReceiver();
    /*
    setup of the receiver module; necessary to run this function after the constructor
    */


    void receiveData();
    /*
    receive data from the remote and set the corresponding m_nrfDataRead
    */

 
    int* getData();
    /*
    return the pointer to an array containing the data received from the remote
    */

    
    void printData();
    /*
    Print the data received from the remote on the serial monitor (function used for debugging purposes)
    */


};


#endif // RADIORECEIVER_H_INCLUDED
