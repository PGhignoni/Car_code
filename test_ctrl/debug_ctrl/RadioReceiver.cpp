#include <SPI.h>
#include "RF24.h"

#include "RadioReceiver.h"


bool RadioReceiver::isListening(){
    return m_flagListening;
}

bool RadioReceiver::isReceiving(){
    return m_flagReceivingData;
}

void RadioReceiver::setupReceiver(){
    Serial.begin(9600);
    while(!m_flagListening)                         // listen until a communication is estabilished
    {
      if (m_radio.begin())
      {                                                // initialize RF24
          m_radio.setPALevel(RF24_PA_MAX);             // set power amplifier (PA) level
          m_radio.setDataRate(RF24_1MBPS);             // set data rate through the air
          m_radio.setRetries(0, 15);                   // set the number and delay of retries
          m_radio.openWritingPipe(m_addresses);        // open a pipe for writing
          m_radio.openReadingPipe(1, m_addresses);     // open a pipe for reading
          m_radio.startListening();                    // start monitoringtart listening on the pipes opened
          m_flagListening=true;
          Serial.println("Start listening remote data");
      }
      else
      {
          Serial.println("Not found the nrf chip");
      }
    }
}

void RadioReceiver::receiveData(){
    delayMicroseconds(1000);
    if (m_radio.available())
    {
        m_flagReceivingData=true ;                                     // if receive the data
            while (m_radio.available())
            {                                                         // read all the data
                m_radio.read(m_nrfDataRead, sizeof(m_nrfDataRead));   // read data
            }
    }
    else
    {
        m_flagReceivingData=false;
    }
}


int* RadioReceiver::getData(){
    auto tmpvec=m_nrfDataRead; // do not edit the content received
    return &(tmpvec[0]);
}


void RadioReceiver::printData(){
    if (m_flagReceivingData)
    {
       Serial.print("P1/P2/X/Y/Z/S1/S2/S3 : ");
            for (int i = 0; i < sizeof(m_nrfDataRead) / 2; i++)
            {
                Serial.print(m_nrfDataRead[i]);
                Serial.print('\t');
            }
        Serial.print('\n');
    }
}
