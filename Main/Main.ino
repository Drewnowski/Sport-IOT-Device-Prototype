/********************************************
********Feather M0 LORA IOT LAB**************
*********************************************/

/* ----------------------------------INCLUDE --------------------------------------- */
/*FALL DETECTION*/
#include "LIS3DHTR.h"
#include <Wire.h>
LIS3DHTR<TwoWire> LIS;
#define WIRE Wire

/*HEARTBEAT*/
#include <Arduino.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "algorithm.h"
#include "max30102.h"

/*LORA*/
#include <lmic.h>
#include <hal/hal.h>


/* ----------------------------------DEFINE--------------------------------------- */
/*FALL DETECTION*/
int led = 13;                             // Integrated LED indicator
float x, y, z, X, Y, Z  = 0;              // Capital letter = actual value, small letter = old value
float deltaX, deltaY, deltaZ = 0;
int fall = 0;                             // Data sent to TTN gateway; 1 if fall detected

/*HEARTBEAT*/
const byte oxiInt = 10;                   // Connected to MAX30102 Interrupt pin
uint32_t aun_ir_buffer[BUFFER_SIZE];      // Infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];     // Red LED sensor data
int var = 0; //fake

/*LORA*/
// EUI -> little-endian format, LSB first. If copying EUI from ttnctl output -> reverse the bytes.
// TTN issued EUIs -> last bytes should be 0xD5, 0xB3,0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// Also in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x3E, 0x97, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// Key should be in big endian format. Or, since it is not really a number but a block of memory, endianness does 
// not really apply. In practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x89, 0xC3, 0x14, 0x49, 0x73, 0x84, 0x35, 0x42, 
                                        0x8B, 0xA0, 0x74, 0x3F, 0xEC, 0xF2, 0xEE, 0xDE };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t payload[7];                // Payload sent to TTN gateway
static osjob_t sendjob;

const unsigned TX_INTERVAL = 1;           // TX every x seconds (might be longer due to duty cycle limitations).

const lmic_pinmap lmic_pins = {           // LoRa Pin mapping. Pin 6 & DIO1 have to be connected
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,                         // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
        Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_JOINED:
          Serial.println(F("EV_JOINED"));
          {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            
            Serial.print("AppSKey: ");
            for (size_t i=0; i<sizeof(artKey); ++i) {
              if (i != 0)
                Serial.print("-");
              printHex2(artKey[i]);
            }
              
            Serial.println("NwkSKey: ");
            for (size_t i=0; i<sizeof(nwkKey); ++i) {
              if (i != 0)
                Serial.print("-");
              printHex2(nwkKey[i]);
            }
            Serial.println();
          }
            
            LMIC_setLinkCheckMode(0);       // Disable link check validation
            break;
            
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE "));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);  
            break;
            
        default:
           Serial.print(F("Unknown event: "));
           Serial.println((unsigned) ev);
           break;
    }                                       // end of switch case
}                                           // end of onEvent()

/*SENSORS AQUISITION & PAYLOAD*/
void do_send(osjob_t* j){
    if (LMIC.opmode & OP_TXRXPEND) {        // = TX/RX job running?
       Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        /*HEARTBEAT DATA*/
        float n_spo2,ratio,correl;          // SPO2 value
        int8_t ch_spo2_valid;               // Show if the SPO2 is valid
        int32_t n_heart_rate;               // HR value
        int8_t  ch_hr_valid;                // Show if the HR is valid
        int32_t i;
        
        for(i=0;i<BUFFER_SIZE;i++)          // 100 samples
        {
          while(digitalRead(oxiInt)==1);    // HEARTBEAT interrupt pin asserts
          maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i)); //read from MAX30102 FIFO
        }
        //Get HR & SpO2 after BUFFER_SIZE samples(ST seconds of samples)
        rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, 
                                            BUFFER_SIZE, 
                                            aun_red_buffer, 
                                            &n_spo2, 
                                            &ch_spo2_valid, 
                                            &n_heart_rate, 
                                            &ch_hr_valid, 
                                            &ratio, 
                                            &correl); 
        /*FALL DETECTION DATA*/
        LIS.getAcceleration(&X, &Y, &Z);
        deltaX, deltaY, deltaZ = X-x, Y-y, Z-z;
        x, y, z = X, Y, Z;
        if(deltaX > 0.9 || deltaY > 0.9 || deltaZ > 0.9){fall = 1;} // 0.9 value was found by testing
        else{fall = 0;}
          
        /*TEMPERATURE DATA*/
        int8_t integer_temperature;
        uint8_t fractional_temperature;
        maxim_max30102_read_temperature(&integer_temperature, &fractional_temperature);
        float temperature = integer_temperature + ((float)fractional_temperature)/16.0;

        // PAYLOAD CREATION
        int shiftTEMP = int(temperature); 
        payload[0] = byte(shiftTEMP);
        payload[1] = shiftTEMP >>8;

        int shiftHR = int(n_heart_rate); 
        payload[2] = byte(shiftHR);
        payload[3] = shiftHR >>8;        
        
        int shiftSPO = int(n_spo2); 
        payload[4] = byte(shiftSPO);
        payload[5] = shiftSPO >>8;        

        payload[6] = byte(fall);
        
        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);  // Next TX scheduled after TX_COMPLETE event     
    }
}

 
/* ----------------------------------SETUP--------------------------------------- */
void setup() {
  pinMode(led,OUTPUT);
  digitalWrite(led,LOW);                    // Turn off internal LED to save power
  
  /*FALL DETECTION*/
LIS.begin(WIRE, 0x19);                      // I²C init
  delay(100);
   
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_10HZ);
  LIS.setPowerMode(POWER_MODE_LOW);         //Low-Power enable
  
/*HEARTBEAT*/
  pinMode(oxiInt, INPUT);                   // pin D10 -> MAX30102 interrupt
  maxim_max30102_init();                    // initialize the MAX30102
  
/*LORA*/
  delay(3000);
  while (! Serial);
  Serial.begin(9600);
  Serial.println(F("Starting"));
  os_init();                                // LMIC init
  
  LMIC_reset();                             // Reset MAC state->Session & pending data transfers discarded
  
  LMIC_setLinkCheckMode(0);                 // Link-check mode & ADR OFF-> ADR complicate testing
  
  LMIC_setDrTxpow(DR_SF7,14);               // Set data rate to Spreading Factor 7,max supported rate for 125kHz 
                                            // channels, minimizes air time & batt power.
  
  do_send(&sendjob);                        //Start job(sending automatically starts OTAA too)
}

/* ------------------------------------LOOP------------------------------------- */
void loop() {
  // LMIC's runloop processor call.Everything based on events and time. 
  // One of the things that will happen is callbacks for transmission complete or received messages. 
  // We use this loop to queue periodic data transmissions. You can put other things here in  
  // the `loop()` routine, but beware that LoRaWAN timing is pretty tight, so if you do more than a few  
  // milliseconds of work, you will want to call `os_runloop_once()` every so often, to keep the radio running.
  os_runloop_once();
}
