/*         cButteTransmit.ino
 *
 * Transmit Only.   See cButteReceive.ino for receive end
 * Repeater monitor for Coffin Butte 147.42 Simplex Repeater
 * W7PUA 11 May 2021  Rev 0.3
 */

// This has several elements that are Teensy LC dependent
#if defined(__MKL26Z64__) // only for Teensy LC

#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>

// Normally the watchdog is disabled at startup.  This removes the startup
// code.  The watchdog will be active with 1024 ms timeout.
// There is no way to start up without the watchdog and then
// enable it later.   WDT code from Mike, W7RIS
extern "C" void startup_early_hook(void) {
    // This is a write once register. Can only be done at startup
    //SIM_COPC = 0;  // disable the watchdog
    SIM_COPC = 12; // enable WDT for 1 second or 1024 MS
    }

/* For reference only, the "External" header  connects as follows:
 * 
 * J3 Pin     Function          Wire Color     9-Pin D Conn
 * ------  ------------------ ---------------    --------
 *   1     DS18820 Ground      Gray                NC
 *   2     Ground              NU
 *   3     Solar Panel Volts   White-Orange        2
 *   4     DS18820  +3.3       Red-Green           NC
 *   5     DS18820 Data        White-Yellow        NC
 *   6     PTT 2-Meter         White-Black         3
 *   7     I Solar -           Green               4
 *   8     I Solar +           Yellow              5
 *   9     2-Meter Sense       White-Green         6
 *   10      NU                Blue                7
 *   11    Ground              Black               1
 *   12    +12V Unreg In       Red                 9
 * 
 * Note J3 pins 1, 4 and 5 go to a separate connector to avoid filtering
 * from the 9-pin D Connector.  This is for the DS18820 Temp Sensor.
 */

// Power Output from 2-m transmitter.  After 10:1 voltage scaling.
// 0.45V 1W, 0.72V 2W, 1.07V 4W, 1.19V 5W, 1.71V 10W
// Po=1.0185*V+2.67494*V*V  Watts

// *****  DS18B20 Temperature sensor, Coffin Butte only  *****

// Setup a oneWire instance for all OneWire devices, pin 8
OneWire oneWire(8);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature externalTempSensor(&oneWire);
// #endif

//  **** Adafruit BME280 Temp Humidity Pressure sensor  ****
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

/* *****       Modtronix inAir9        ********
 *                Teensy      inAir9   Function
 *                 GND----------GND   (ground in)
 *                 3V3----------3.3V  (3.3V in)
 *       INT 0 pin D2-----------D0    (interrupt request out DIO0)
 *         RST pin D9-----------RT    (Reset)
 *          SS pin D10----------CS    (CS chip select in)
 *        MOSI pin D11----------SI    (SPI Data in)
 *        MISO pin D12----------SO    (SPI Data out)
 *         SCK pin D13----------CK    (SPI clock in)
 */
#define RFM95_RST     9
#define RFM95_CS      10
#define RFM95_INT     2
#define ONE_WIRE_BUS  8

#define SW1 3
#define SW2 4
#define BEEPER 5
#define PTT 6
#define LED 17

/* All messages sent and received by this RH_RF95 Driver conform to this packet format:
 *  - LoRa mode:
 *  - 8 symbol PREAMBLE
 *  - Explicit header with header CRC (default CCITT, handled internally by the radio)
 *  - 4 octets HEADER: (TO, FROM, ID, FLAGS)10];
 *  - 0 to 251 octets DATA
 * - CRC (default CCITT, handled internally by the radio)
 */
RH_RF95 rf95(RFM95_CS, RFM95_INT);   // from Xecuter

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];  // 251
uint8_t len = sizeof(buf);
bool need2MeterPower = true;

uint32_t t0 = 0;
uint32_t count60Sec;
uint32_t timeCount;       // In minutes, big
uint16_t hourCount = 0;   // Rolls over in 7-1/2 years
uint16_t numDays;
bool     needDataCollect = false;
bool     needDataSend = false;

// 
int16_t analogIn[10];
int8_t  analogPin[] = {A0, A1, A2, -1, -1, -1, -1, A7, A8, A9}; // -1 for non-analog pins
float   analogMultiplier[10] = {0.0102286f, 0.00926826f, 0.0088689f, 0.0f,  0.0f,  0.0f,  0.0f, 0.001f, 0.00088148f, 0.01546f};
int16_t analogOffset[10] =     {         1,          0,          0,    0,     0,     0,     0,      1,      0,     2027};     
char    analogShortName[10][5] = { "Solr",    "+12V",      "5.0V", "    ", "    ", "    ", "    ", "PLor", "P2m ", "IBat"};
int16_t loraPower;
int16_t power2Meters;
// LoRa power, integer value from detector vs Integer ADC, P
// gives SetPtdBm = 4.5469 + 0.02073*P - 0.00000809*P*P

#define CB_VERSION 03
uint8_t cbStatus = 0;  // Use up to ls 6 bits as status bits (63 decimal, max). LS 3 as restart count
/* 86 char in LoRa send line c[]
0,3   Version, status
5,8   Minutes, ms
10,13 Minutes, ls
15,18 nu
20,23 Texternal
25,28 Tinternal
30,33 Hinternal
35,38 A0
40,43 A1
45,48 A2
50,53 A7
55,58 A8
60,63 A9
65,68 AtmosPress
70,73 Restart Count 0 to 9999, from EEPROM
75,80 nu
80,83 nu
84,85 Extra for sprintf()
*/
char c[86];   // The transmitted data by LoRa.
float   externalTempC;
uint16_t restartCount;
uint32_t tt;

void setup()  {
  restartCount = 1 + 256*EEPROM.read(20)+ EEPROM.read(21);

  // To reset the restartCount, uncomment the next line and recompile
  // this INO.  BE SURE to next put the comment slashes back in and recompile again.
  //    EEPROM.write(20, 0);  EEPROM.write(21, 0);

  if(restartCount < 10000)  {
     EEPROM.write(20, (uint8_t)(restartCount/256));
     EEPROM.write(21, (uint8_t)(restartCount - 256*(restartCount/256)));
     }


  wdtReset();

  // Digital pins
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(BEEPER, OUTPUT);
  digitalWrite(BEEPER, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);  // Off
  pinMode(PTT, INPUT);

  Serial.begin(9600);
  for(int kk=0; kk<10; kk++)  {
     delay(100);
     wdtReset();
     }
  Serial.println("Coffin Butte Simplex Repeater Monitor");
 
  // Restart by getting last day count and adding 720 min (12 hours)
  // as a guess.  The day count is in EEPROM.
  numDays = getDay();
  timeCount = 1440*numDays + 720;  // In minutes
  Serial.print("Start up time estimate, in days = ");
  Serial.println(0.00069444444f*(float)timeCount);

  // To reset the clock, not normally ever done, un-comment next
  // four lines, run once and then comment again.
  //    timeCount = 0;
  //    hourCount = 0;
  //    numDays = 0; 
  //    Serial.println(" *** CLOCK WAS RESET  ***";    

  externalTempSensor.begin();  // External temp sensor
  wdtReset();

  loraReset();  // Reset pin low and then high
  wdtReset();

  // Initialize the LoRa SX1276
  if (!rf95.init())
    Serial.println("LoRa init failed");
  else
    Serial.println("LoRa init was successful");
  wdtReset();

  Serial.println("Transmit Frequency = 923.30 MHz");
  rf95.setFrequency(923.30f);

  /* There are four RH_RF95 preset options for modem settings.  We use
   *    Bw125Cr48Sf4096
   *    Bw = 125 kHz,   Cr = 4/8, Sf = 4096chips/symbol, CRC on.  (Slow + long range)
   */
  //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); // <<<<<<<<<<<<<<<

  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);

  // Info: Total current is about 65 mA at +10 dBM and 90 mA at +18 dBm set points.
  Serial.println("Transmit Power 17 dBm");
  rf95.setTxPower(17, false);  // Modtronix inAir9 uses PA_BOOST, so use false
  wdtReset();
  delay(100);

  // This is the blank packet to send
  for(int k=0; k<86; k++)  c[k] = '0';
  for(int k=0; k<17; k++)  c[5*k-1] = ',';
  c[85] = 0;
  sprintf(&c[70], "%.4d", restartCount); c[74]=',';  // Current number of restarts
  Serial.print("Number of Restarts of this program: ");
  Serial.println(restartCount);

  Wire.setSDA(18);  // Initialize I2C Wire
  Wire.setSCL(19);
  Wire.begin();
  wdtReset();

  // Adfruit BME280 Temp/Humidity/Pressure Sensor
  int kk = 0;
  if ( !bme.begin() )
     Serial.println("Could not find a valid BME280 sensor!");
  wdtReset();
  // Print summary of internal BME280 sensor.
  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  //  ***** Analog Voltages  *****
  analogReadResolution(12);
  analogReadAveraging(8);

  count60Sec = millis();
  timeCount = 0;
  needDataSend = false;
  needDataCollect = true;
  tt=(uint32_t)millis();
  wdtReset();
}

void loop()  {
  uint16_t txGood0;

  cbUpdateClock();
  wdtReset();

  if(needDataCollect) {
     uint16_t vs = (256*CB_VERSION) | (0X3F & cbStatus);
     sprintf( &c[0], "%.4d", vs);   c[4]=',';
     Serial.print("Version and Status 0X");
     if(vs<4096) Serial.print('0');
     Serial.println( (256*CB_VERSION | (0X3F & cbStatus)), HEX);

     Serial.print("Number of Restarts of this program: ");
     Serial.println(restartCount);
     wdtReset();

     cbVoltageRead();
     wdtReset();
     cbExternalTemperature();
     wdtReset();
     cbInternalTempHudity();
     Serial.println("");
     wdtReset();
     needDataCollect = false;  // Have new data
     needDataSend = true;      // Send it
     }

  if(needDataSend)  {
     // Transmit LoRa
     Serial.print("Packet:"); Serial.println(c);
     if( rf95.send((uint8_t*)c, sizeof(c)) ) {
        tt = millis();
        digitalWrite(LED, LOW);
        txGood0 = rf95.txGood();
        Serial.println("LoRa transmit begin is successful");
        delay(10);
        loraPower = analogRead(A7); 
        wdtReset();
        while(1)  {
           // Serial.print( rf95.txGood() );
           // Serial.print(" time= "); Serial.println( (uint32_t)millis() - tt );
           wdtReset();
           delay(10);
           if ((rf95.txGood()-txGood0) > 0)
              break;
           // and a safety timeout at 10 seconds and allow for wrap around
           if( (millis() > tt+10000L) )
               break;
           }
        Serial.print("Lora transmit complete, transmit time, seconds = ");
        Serial.println(  (0.001f*(float)(millis() - tt)), 2);
        digitalWrite(LED, HIGH);
        //  delay(2000);  // TEST WDT <<<<<
        needDataSend = false;  // Just sent it
        }
      else
        Serial.println("rf95.send() failed");  // More detail??
      delay(10);
      }  //End, need data send

      // Watch PTT line and if low, get transmit power.  Do once until sent
      if(need2MeterPower && digitalRead(PTT)==LOW)  {
         delay(50);       // Wait for full output
         power2Meters = analogRead(A8);
         need2MeterPower = false;   // Only once per data transmit
         }

  }   // End, loop()

void cbUpdateClock()  {
    uint32_t ms = millis();
    char ch[12];
    if((ms + 10) < count60Sec)  {
      count60Sec = millis(); // millis() has rolled over
      // count60Sec is 32-bits, so here about every 50 days
      return;
      }
    if( (ms - count60Sec) > 60000)  {     // a new minute
      timeCount++;
      if(timeCount - 1440*numDays  > 1440) {
         putDay( (uint16_t) (timeCount/1440L) );
         Serial.println("Added day");
         numDays++;
         }
      count60Sec = ms;
      needDataCollect = true;
      // Create two 4-digit "number of minutes" for LoRa transmission
      uint32_t tms = timeCount/10000L;
      Serial.print("Time minutes = ");
      Serial.println(timeCount);
      sprintf(&c[5], "%.4d", (uint16_t)tms);
      sprintf(&c[10], "%.4d", (uint16_t)(timeCount - 10000L*tms));
      c[9]=',';  c[14]=',';
      }
    }    // end, cbUpdateClock()

// getDay() and putDay(d)
uint16_t getDay(void)  {
   return  (256*EEPROM.read(17)+ EEPROM.read(18));
   }

// in order to not destroy EEPROM, we write this once a day.
void putDay(uint16_t nDay)  {
   EEPROM.write(17, (uint8_t)(nDay/256));
   EEPROM.write(18, (uint8_t)(nDay - 256*(nDay/256)));
   }

// 35,38 A0;  40,43 A1;  45,48 A2;  50,53 A7;  55,58 A8;  60,63 A9
void cbVoltageRead()  {
   for(int i=0;  i<10; i++)  {
       if(analogPin[i] >= 0)  {
          analogIn[i] = analogRead(analogPin[i]);
          Serial.print("Voltage A"); Serial.print(i);
          Serial.print(" (");; Serial.print(analogShortName[i]); Serial.print(") = ");

          if(i==7)  {
              analogIn[7] = loraPower;  // Measured when LoRa was transmitting
              float P = (float)loraPower;
              // Show 4.5 dBm for no output, but is accurate for > 6 or 8 dBm
              Serial.print(4.5469f + 0.02073f*P - 0.00000809f*P*P, 1); Serial.println(" dBm");
              }
          else if(i==8)  {
              analogIn[8] = power2Meters;
              float Vp = 0.0008815f*(float)power2Meters;  // Measured last 2-m xmit
              Serial.print(1.0185*Vp + 2.67494*Vp*Vp, 2);    Serial.println(" Watts");
              need2MeterPower = true;   // Get new measurement, if available
              }
          else
              Serial.println( (analogMultiplier[i]*(float)(analogIn[i]-analogOffset[i])), 3);

          if(i<=2)      {sprintf(&c[35+5*i], "%.4d", analogIn[i] ); c[39+5*i]=',';}
          else if(i>=7) {sprintf(&c[15+5*i], "%.4d", analogIn[i] ); c[19+5*i]=',';}
       }
     }
  }

// External temp goes to bytes c[20] to c[23]
void cbExternalTemperature()  {
  //   *****   External temperature sensor   *****
  externalTempSensor.requestTemperatures(); // Send cmd
  // Get the temperature from the first (only) sensor
  externalTempC = externalTempSensor.getTempCByIndex(0);
  sprintf(&c[20], "%.4d", (int16_t)(1000.0f + 10.0f*externalTempC));
  c[24]=',';
  // Check if reading was successful
  if(externalTempC==DEVICE_DISCONNECTED_C)
     {
     //Error response - needed? it is obvious from temp
     }
  else
     {
     Serial.print("External Temp = ");
     Serial.print(externalTempC, 1);
     Serial.println(" deg C");
     }
  }

void cbInternalTempHudity()  {
   sensors_event_t temp_event, pressure_event, humidity_event;
   bme_temp->getEvent(&temp_event);
   bme_pressure->getEvent(&pressure_event);
   bme_humidity->getEvent(&humidity_event);

   sprintf( &c[25], "%.4d", (int16_t)(1000.0f + 10.0f*temp_event.temperature) );
   sprintf( &c[30], "%.4d", (int16_t)(10.0f*(0.5 + humidity_event.relative_humidity)) );
   sprintf( &c[65], "%.4d", (int16_t)(0.5 + pressure_event.pressure) );
   c[29]=',';  c[34]=',';  c[69]=',';

   Serial.print("Internal Box Temp, celsius ");
   Serial.println( bme.readTemperature() );
   Serial.print("Barometric Pressure, uncorrected mB ");
   Serial.println( 0.01f*bme.readPressure() );
   Serial.print("Relative Humidity, percent ");
   Serial.println( bme.readHumidity() );
   }

void cbRFPower()  {

  }

void loraReset(void)  {
  // Manual Reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  }

// Reset or 'kick' the watchdog timer to prevent a reset of the device.
// From W7RIS work.
void wdtReset(void) {
  __disable_irq();  // noInterrupts(); different?
  SIM_SRVCOP = 0x55;
  SIM_SRVCOP = 0xAA;
  __enable_irq();
  }

#else
  #error Unsupported platform for the Teensy LC Watchdog registers!
#endif
