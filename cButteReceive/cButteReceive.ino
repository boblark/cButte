/*         cButteReceive.ino   ver 0.2
 * Repeater monitor for Coffin Butte 147.42 Simplex Repeater
 * WARNING - do NOT use "Smallest Code" option under tools (sprintf issues)
 * W7PUA 10 Jan 2021; Rev 0.2 22 May 2021
 */

#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <SerialFlash.h>

/* We need minimum LCD functions, so build minimum library. Pieces taken
 * from LCD_I2C_Teensy36 that also has I2C  parts that collide with Wire.h.
 * So, this is simplist.  A few needed defines:
 */
#define LCD_CLEARDISPLAY 0x01
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_DISPLAYON 0x04
#define LCD_CURSOROFF 0x00
#define LCD_BLINKOFF 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00
#define LCD_FUNCTIONSET 0x20
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80
#define lcdEn B00000100  // Enable bit
#define lcdRw B00000010  // Read/Write bit   nu
#define lcdRs B00000001  // Register select bit

#define LED_OFF digitalWrite(led,HIGH)
#define LED_ON  digitalWrite(led,LOW)

#define DISPLAY_SERIAL 1
#define DISPLAY_LCD  2

/* ********   LoRa     Modtronix inAir9        ********
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

// TODO: This can take a SPI hardware description to avoid collisions with
// the secondd SD SPI hardware port. See
// https://forum.pjrc.com/threads/41238-Maybe-updates-for-SPI-Wire-ili9341_t3-libraries
// post 130616.   For now just don't use the SD card.
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define ONE_WIRE_BUS  8

// And some variables
uint8_t lcdDisplayfunction;
uint8_t lcdDisplaycontrol;
uint8_t lcdDisplaymode;
uint8_t lcdBacklightval;
char    lcdData[2][17]; // Allow for possible \0's (not needed)

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];  // 251
uint8_t len = sizeof(buf);
int led = 17;
float sc5V, sc12V;    // Local receive end measurementscbStatus

/* Errors  Value    Transmit end           Receive End
 * bit 0    1     Voltages, currents   Voltages, Currents
 * bit 1    2     LoRa                 Lora
 * bit 2    4     Environment          Data Quality
 * bit 3    8     Other                Other              */
uint8_t cbError = 0;  // All OK, bits 0 to 2 for error categories
uint8_t scError = 0;  // ditto

uint32_t t0 = 0;
int16_t analogIn[10];
int8_t  analogPin[] = {A0, A1, A2, -1, -1, -1, -1, A7, A8, A9}; // -1 for non-analog pins
float   analogMultiplier[10] = {0.0102286f, 0.00926826f, 0.0088689f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f, 0.0008815f, 0.01546f};
int16_t analogOffset[10] =     {         1,          0,          0,    0,    0,    0,    0,      1,      1,     2027};     
char    analogShortName[10][5] = { "Solr",    "+12V",      "5.0V", "   ", "    ", "    ", "  ", "PLor", "P2m ", "IBat"};

/* 86 char in LoRa send line c[]
0,3   Version, status
5,8   Minutes, ms
10,13 Minutes, ls
15,18 nu
20,23 Texternal
25,28 Tinternal
30,33 Hinternal
35,38 A0 Solar Voltage
40,43 A1 Battery Voltage
45,48 A2 3.3 Voltage
50,53 A7 LoRa Power
55,58 A8 2-meter Power
60,63 A9 Battery Current
65,68 nu
70,73 Number of program restarts at C. Butte
75,80 nu
80,83 Barometeric Pressure
84,85 Extra for sprintf()
*/
char c[86];
char cSave[16][86];
int indexSave = 0;
uint32_t timeSave = 0;
float   externalTempC;
uint16_t decodeFails;
uint32_t minutesTime;

#define FREQUENCY_BASE 923.3
float frequencyRMHz = FREQUENCY_BASE;
float frequencyErrorMHz = 0.0f;
bool outOfLock = false;

uint8_t charOK[] = {0X1C, 0X14, 0X14, 0X1C, 0X05, 0X06, 0X05, 0X05};

//  ****************   SETUP()   *******************
void setup()  {
  pinMode(led, OUTPUT);
  LED_ON;

  Serial.begin(9600);
  delay(1000);
  Serial.println(" RH_RF95 Lora Receiver ver 0.2");

  for(int i=0; i<86; i++) {
    c[i]=(char)'-';
    for(int j=0; j<16; j++)
       cSave[j][i] = ' ';
    }
  c[85] = 0;

#if 0
  // TODO: Turn this into SD Card logging w/o breaking the LoRa SPI port.
  // Teensy LC can use alternate pins for these 3 SPI signals.
  SPI1.setMOSI(0);
  SPI1.setMISO(1);
  SPI1.setSCK(20);
  // return true if "pin" has special chip select capability
  // Serial.print("Is CS? : "); Serial.println(SPI1.pinIsChipSelect(6));

  pinMode (6, OUTPUT);      // set the slaveSelectPin as an output:
  pinMode (6, HIGH);
  SPI1.begin();           // initialize SPI:
#endif

  decodeFails=0;
  loraReset();  // Reset pin low and then high
  // Initialize the LoRa SX1276
  if (!rf95.init())
     Serial.println("LoRa init failed");
  else
     Serial.println("LoRa init was successful");
  rf95.setFrequency(frequencyRMHz);

/* Bw125Cr45Sf128 = 0,    ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
   Bw500Cr45Sf128,            ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
   Bw31_25Cr48Sf512,      ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
   Bw125Cr48Sf4096,           ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range  */
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);

  Wire.setSDA(18);  // Initialize I2C Wire
  Wire.setSCL(19);
  Wire.begin();

  lcdBegin_cb();
  lcdClear_cb();
  delay(4000);

  lcdCreateChar_cb(0, charOK);  // Custom character, small 'o' and small 'k'
  strcpy( &lcdData[0][0], "1               " );
  strcpy( &lcdData[1][0], "2               " );
  lcdPrintLine(0);
  lcdPrintLine(1);
  }

//    *************   LOOP()   ***********
void loop()  {
   // This puts a slight flicker to the LED when the loop is running
   LED_OFF;
   delay(10);
   LED_ON;
   delay(200);
   if ( rf95.available() )  {
      // Should be a message for us now
      if (rf95.recv(buf, &len))  {
         Serial.println("\n\nCoffin Butte 147.42 MHz Simplex Repeater Report  Rev 0.1");
         // RH_RF95::printBuffer("Receive occurred, data = ", buf, len);
         Serial.println("Data as Received: ");
         Serial.println((char*)buf);

        Serial.print("+12 Supply Voltage: ");
        // This may be wrong K for some boards?
        sc12V = 0.035353f*(float)analogRead(A1);
        Serial.println(sc12V, 1);
        if(sc12V<11.0f || sc12V>16.0f)  scError |= 1;

        Serial.print("+5 Internal Voltage: ");
        sc5V = 0.035353f*(float)analogRead(A2);
        Serial.println(sc5V, 1);
        if(sc5V<4.5f || sc5V>5.5f)  scError |= 1;

        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);

        scSNR(DISPLAY_SERIAL | DISPLAY_LCD, 0, 0);

         Serial.print("Decode failures = ");
         Serial.println(decodeFails);
         // rf95.printRegisters();
         frequencyErrorMHz = 0.001f*FrequencyErrorKHz();
         Serial.print("Frequency Fr-Ft difference, kHz: ");
         Serial.println(1000.0f*frequencyErrorMHz, 2);
         frequencyAFC(frequencyErrorMHz);
         }
      else  {
         decodeFails++;
         Serial.print("Receive available, but Defodefailure.");
         Serial.print("  Cumulative Decode failures = ");
         Serial.println(decodeFails);
         }

      // Set errors to zero and then check
      cbError = 0;
      scError = 0;
      lcdClear_cb();
      // Now process the line of numbers
      Serial.println("Report from Repeater Site:");
      timeDate(DISPLAY_SERIAL, 0, 0);
      scVoltsSolar(DISPLAY_SERIAL | DISPLAY_LCD, 1, 0);
      scCurrentBattery(DISPLAY_SERIAL | DISPLAY_LCD, 1, 6);
      scVolts12(DISPLAY_SERIAL, 1, 0);
      scVolts5(DISPLAY_SERIAL, 1, 0); 
      scVoltsRFLoRa(DISPLAY_SERIAL, 0, 0);
      scVoltsRF2mtr(DISPLAY_SERIAL, 0, 0);
      scExtTemp(DISPLAY_SERIAL, 0, 0);             // LCD Upper Left
      scInternalTemp(DISPLAY_SERIAL | DISPLAY_LCD, 0, 5);
      scInternalHumidity(DISPLAY_SERIAL | DISPLAY_LCD, 0, 11);
      scAtmosPress(DISPLAY_SERIAL, 0, 0);
      statusVersion(DISPLAY_SERIAL, 0, 15);
      scRestartCount(DISPLAY_SERIAL, 0, 0);
      setErrorCharacters();

      lcdPrintLine(0);     // Update the 2x16 display
      lcdPrintLine(1);

      // Every 3 hours, save a sample of the data in a 16 entry circular buffer.
      if(minutesTime - timeSave >= 180) {  // Every 3 hours
          int j = ++indexSave & 0X0F;
          for(int ii=0; ii<86; ii++)
              cSave[j][ii] = c[ii];
          indexSave = j;
          timeSave = minutesTime;
          }
      }      // End, if LoRa data available
   }   // End loop()

/* Errors  Value    Transmit end           Receive End
 * bit 0    1     Voltages, currents   Voltages, Currents
 * bit 1    2     LoRa                 Lora
 * bit 2    4     Environment          Data Quality
 * bit 3    8     Other                Other 
 * 
              Two Status Characters
Repeater Site, Upper Right Status Character, error conditions:
  Solar Panel Voltage             1  >40
  Battery Current Amps            1  <-1 or >20
  Supply Voltage from Battery     1  <11.5 or >16
  5V Supply Voltage for Teensy    1  <4.5 or >5.5
  LoRa measured RF Power          2  <10
  2-meter RF Power                8  <0.5 or >7
  External Temperature, Celsius   -  None
  Internal Temperature, Celsius   4  >50C
  Internal Humidity, percent      4  >85
  Atmospheric Pressure in-Hg      -  None
  Restart Count                   8  > 1 per day
Receiver Site, Lower Right Status Character, error conditions
  +12 Supply Voltage:             1  <11 or >16
  +5 Internal Voltage:            1  <4.5 or >5.5
  RSSI:                           -  None for now
  SNR dB:                         4  <7
  Decode failures                 4 >1 per day
  Frequency Fr-Ft diff, kHz:      8 >5 kHz
  Current Receiver Tuning, MHz:   8 off more than 50 kHz
*/

void setErrorCharacters(void)  {
  if(cbError == 0)
     lcdData[0][15] = 0;
  else
     lcdData[0][15] = hexChar2ASCII(cbError);
  if(scError == 0)
     lcdData[1][15] = 0;
  else
     lcdData[1][15] = hexChar2ASCII(scError);
}

uint8_t hexChar2ASCII(uint8_t e)  {
  e &= 0X0F;
  if(e<=9) return e+0X30;
  return e+0X37;
}

// From spiRead() of  <RHSPIDriver.h>
float FrequencyErrorKHz(void)  {
#define REG_FREQ_ERROR  0X28
  int32_t deltaFreq;
  float   deltaFreqf;
  uint8_t reg = REG_FREQ_ERROR;

  deltaFreq = (int32_t)rf95.spiRead(reg) & 7;
  deltaFreq <<= 8L;
  deltaFreq += (int32_t)rf95.spiRead(reg+1);
  deltaFreq <<= 8L;
  deltaFreq += (int32_t)rf95.spiRead(reg+2);
  if (rf95.spiRead(reg) & 8)
     deltaFreq = deltaFreq - 524288;
  // 0.25 for 125 kHz spread = 125/500; 0.001 for kHz
  deltaFreqf = 0.001f*0.25f*0.524288f*(float)deltaFreq;
  if (deltaFreqf<-40.0f || deltaFreqf>40.0f)  cbError |= 2;
  return deltaFreqf;
  }

// Single pole filtered AFC
void frequencyAFC(float freqErrorMHz)  {
  frequencyRMHz -= 0.1f*freqErrorMHz;
  rf95.setFrequency(frequencyRMHz);
  Serial.print("Current Receiver Tuning, MHz: ");
  Serial.println(frequencyRMHz, 3);
  }

// Convert the 4-letter ASCII to an integer
uint16_t lora2int(int index) {
    char s[5];
    s[0]=buf[index]; s[1]=buf[index + 1]; s[2]=buf[index + 2]; s[3]=buf[index + 3]; s[4]=0;
    return (uint16_t)atoi(s);
    }

// The first group of 4 ASCII characters is different from the rest.
// It is full 16-bit integer sent as 4 hex characters.  
// SSVV with SS being 8-bits of status and VV being the version (00, FF).
void statusVersion(int display, int ln, int pos)  {
    char cc[5];
    uint8_t stat, ver;

    for(int nn=0; nn<4; nn++)
       cc[nn] = buf[nn];  // No offset in buf[] for this one
    cc[4] = 0;
    ver = (uint8_t)atoi(&cc[2]);
    if(display & 1) {
       Serial.print("  Version of Transmit INO: 0X");
       Serial.println(ver);
       }

    cc[2] = 0;     // New end spot
    stat = (uint8_t)atoi(&cc[0]);
    if(display & 1) {
       Serial.print("  Status of Transmit end: 0X");
       Serial.println(stat, HEX);
       }

    if(display & 2)  {       // Status on right edge
       lcdData[0][15] = buf[0];
       lcdData[1][15] = buf[1];
       }
    }

// Time and  Date comes as cumulative minutes at hilltop
// This is saved every 24 hours to EEPROM, and so a reset
// at the transmit end will produce a backward jump in minutes.
// 2-digital words, (0, 9999) for a maximum 99,999,999 minutes.
void timeDate(int display, int ln, int pos)  {
    char mins[10];

    // Get two 4-digit numbers to string.
    for(int nn=0; nn<4; nn++)  {
       mins[nn] = buf[5 + nn];
       mins[nn + 5] = buf[10 + nn];
       }
    mins[4] = ' ';
    mins[9] = 0;

    minutesTime = 10000UL*(uint32_t)lora2int(5) + (uint32_t)lora2int(10);
    // Serial print as two space separated 4 digit numbers
    if(display & 1) {
       Serial.print("  Time in minutes:  ");
       Serial.println(mins);
       }
    // LCD print 
    if(display & 2) {
       sprintf(&lcdData[ln][pos], "%s", mins);
       lcdData[ln][pos + 8] = ' ';   // Space separator
       }
   }

// Solve "sprintf with %f" problems.  From Michael Meissner:
// https://forum.pjrc.com/threads/55152-Teensy-LC-printf-float-options Post #11
// Convert a floating point number to a string with a %<m>.<n>f format.
// On the Teensy, if you optimize for space, a smaller library is used that
// does not do have %f, %g, etc. formats.
void float2String (float number, char *buffer, int fract)  {
    bool negative   = (number < 0.0f);
    float absNumberf   = fabsf (number);

    if (fract == 0) {    // Removed trailing decimal pt.  Make optional?
      absNumberf += 0.5f;
      sprintf (buffer, "%2d", (int) (negative ? -absNumberf : absNumberf));
      }

    else if (fract == 1) {
      int number_x10    = (int) ((absNumberf * 10.0f) + 0.5f);
      int number_main   = number_x10 / 10;
      int number_fract  = number_x10 % 10;
      if (negative)
         sprintf (buffer, "-%d.%c", number_main, number_fract + '0');
      else
         sprintf (buffer, "%2d.%c", number_main, number_fract + '0');
      }

    else if (fract == 2) {
       int  number_x100  = (int) ((absNumberf * 100.0f) + 0.5f);
       int  number_main  = number_x100 / 100;
       int  number_fract = number_x100 % 100;
       char digit1   = ((number_fract / 10) % 10) + '0';
       char digit2   = ((number_fract / 1)  % 10) + '0';
       if (negative)
          sprintf (buffer, "-%d.%c%c", number_main, digit1, digit2);
       else
          sprintf (buffer, "%2d.%c%c", number_main, digit1, digit2);
       }

    else {     // fract==3
       int  number_x1000 = (int) ((absNumberf * 1000.0f) + 0.5f);
       int  number_main  = number_x1000 / 1000;
       int  number_fract = number_x1000 % 1000;
       char digit1   = ((number_fract / 100) % 10) + '0';
       char digit2   = ((number_fract / 10)  % 10) + '0';
       char digit3   = ((number_fract / 1)   % 10) + '0';
       if (negative)
          sprintf (buffer, "-%d.%c%c%c", number_main, digit1, digit2, digit3);
       else
          sprintf (buffer, "%2d.%c%c%c", number_main, digit1, digit2, digit3);
       }
    }

// Each routine processes one of the data items, sending it to the USB-Serial
// link and displaying. Variable display bit 0 is USB-Serial, bit 1 is LCD.
void scVolts5(int display, int ln, int pos)
    {
    float vi = analogMultiplier[2]*(float)(lora2int(45) - analogOffset[2]);
    if(display & 1) {
       Serial.print("  5V Supply Voltage for Teensy = ");
       Serial.println(vi, 3);
       }
    if(display & 2) {
       lcdData[ln][pos] = 'V';
       lcdData[ln][pos+1] = '5';
       float2String(vi, &lcdData[ln][pos+2], 2);
       lcdData[ln][pos + 7] = ' ';
       }
    if(vi<4.5 || vi>5.5)  cbError |= 1;
    }

void scVolts12(int display, int ln, int pos)
    {
    float vi = analogMultiplier[1]*(float)(lora2int(40) - analogOffset[1]);
    if(display & 1) {
       Serial.print("  Supply Voltage from Battery = ");
       Serial.println(vi, 2);
       }
    if(display & 2) {
       lcdData[ln][pos] = 'V';
       lcdData[ln][pos+1] = '1';
       lcdData[ln][pos+2] = '2';
       float2String(vi, &lcdData[ln][pos+4], 1);
       lcdData[ln][pos + 8] = ' ';
       }
    if(vi<11.0f || vi>16.0f)  cbError |= 1;
    }

void scVoltsSolar(int display, int ln, int pos)
    {
    float vi = analogMultiplier[0]*(float)(lora2int(35) - analogOffset[0]);
    if(display & 1) {
       Serial.print("  Solar Panel Voltage = ");
       Serial.println(vi, 2);
       }
    if(display & 2) {
       lcdData[ln][pos] = 'V';
       lcdData[ln][pos+1] = 's';
       float2String(vi, &lcdData[ln][pos+2],0);
       lcdData[ln][pos + 4] = ' ';
       }
    if(vi>40.0f)  cbError |= 1;
    }

void scVoltsRF2mtr(int display, int ln, int pos)
    {
    float vi = analogMultiplier[8]*(float)lora2int(55);
    float power2M = 1.0185f*vi + 2.67494f*vi*vi;
    if(display & 1) {
       Serial.print("  2-meter RF Power = ");
       Serial.print(power2M, 2);
       Serial.println(" Watts");
       }
   if(display & 2) {
       lcdData[ln][pos] = 'P';
       lcdData[ln][pos+1] = '2';
       float2String(vi, &lcdData[ln][pos+2],0);
       lcdData[ln][pos + 4] = ' ';
       }
    if(power2M<0.5f || power2M>7.0f)  cbError |= 8;
    }

void scVoltsRFLoRa(int display, int ln, int pos)
    {
    float vi = (float)lora2int(50);
    // Show 4.5 dBm for no output, but is accurate for > 6 or 8 dBm
    float P = 4.5469f + 0.02073f*vi - 0.00000809f*vi*vi;
    if(display & 1) {
       Serial.print("  LoRa measured RF Power = ");
       Serial.print(P, 1);
       Serial.println(" dBm");
       }
   if(display & 2) {
       lcdData[ln][pos] = 'P';
       lcdData[ln][pos+1] = 'L';
       float2String(vi, &lcdData[ln][pos+2],0);
       lcdData[ln][pos + 4] = ' ';
       }
    if(P<10.0f)  cbError |= 2;
    }

void scCurrentBattery(int display, int ln, int pos)
    {
    float vi = analogMultiplier[9]*(float)(lora2int(60) - analogOffset[9]);
    if(display & 1) {
       Serial.print("  Battery Current Amps = ");
       Serial.println(vi, 3);
       }
    if(display & 2) {
       lcdData[ln][pos] = 'I';
       lcdData[ln][pos+1] = 'b';
       float2String(vi, &lcdData[ln][pos+2], 2);
       lcdData[ln][pos + 7] = ' ';
       }
    if (vi<-1.0f || vi>20.0f)  cbError |= 1;
    }

void scExtTemp(int display, int ln, int pos)
   {
   int ivi = lora2int(20) - 1000;
   float vi = 0.1f*(float)ivi;
   if(display & 1) {
      Serial.print("  External Temperature, Celsius = ");
      Serial.println(vi, 1);
      }
   if(display & 2) {
     lcdData[ln][pos] = 'T';
     sprintf(&lcdData[ln][pos+1], "%3d", ivi/10);
     lcdData[ln][pos + 4] = ' ';
     }
  // Errors, what would they be?
  }

void scSNR(int display, int ln, int pos)
   {
   uint8_t snr = rf95.spiRead(0X19);
   if(display & 1) {
      Serial.print("SNR dB: ");
      Serial.println(snr);
      }
   if(display & 2) {
      lcdData[ln][pos] =   'S';
      lcdData[ln][pos+1] = 'N';
      sprintf(&lcdData[ln][pos+2], "%2d", snr);
      lcdData[ln][pos + 4] = ' ';
      }
  if(snr<7)  scError |= 4;
  }

void scRestartCount(int display, int ln, int pos)
    {
    //static uint16_t lastRestartCount = 9999;
    //static uint16_t RestartDisplayDecay = 0;

    uint16_t ct = lora2int(70);
    if(display & 1) {
       Serial.print("  Restart Count = ");
       Serial.println(ct, 4);
       }
    if(display & 2) {
       lcdData[ln][pos] = 'R';
       sprintf(&lcdData[ln][pos+1], "%4d", ct);
       lcdData[ln][pos + 5] = ' ';
       }
    // TODO A count algorithm for error display???
    }

void scInternalTemp(int display, int ln, int pos)
   {
   int ivi = lora2int(25) - 1000;
   float vi = 0.1f*(float)ivi;
   if(display & 1) {
      Serial.print("  Internal Temperature, Celsius = ");
      Serial.println(vi, 1);
      }
   if(display & 2) {
     lcdData[ln][pos] = 'T';
     lcdData[ln][pos+1] = 'i';
     sprintf(&lcdData[ln][pos+2], "%3d", ivi/10);
     lcdData[ln][pos + 5] = ' ';
     }
   if(vi>50.0f)  scError |= 4;
   }

void scInternalHumidity(int display, int ln, int pos)
   {
   int ivi =lora2int(30);
   float vi = 0.1f*(float)ivi;
   if(display & 1) {
      Serial.print("  Internal Humidity, percent = ");
      Serial.println(vi, 2);
      }
   if(display & 2) {
     lcdData[ln][pos] = 'H';
     sprintf(&lcdData[ln][pos+1], "%2d", ivi/10);
     lcdData[ln][pos + 3] = ' ';
     }
   if(vi>85.0f)  scError |= 4;
   }

// Pressure drops 1.07 in-Hg per 1000 ft.  CButte abt 700 ft. (.75" adj)
void scAtmosPress(int display, int ln, int pos)
   {
   float vi = 0.02953f*(float)(lora2int(65)) + 0.75f;
   if(display & 1) {
      Serial.print("  Atmospheric Pressure in-Hg = ");
      Serial.println(vi, 2);
      }
   if(display & 2) {
      sprintf( &lcdData[ln][pos], "Hi=%2.0f", vi );
      lcdData[ln][pos + 3] = ' ';
      }
   // Interesting, but no error conditions
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

/* ***************************************************************
  ****  LCD mini library.  16x2 only and uses Wire for I2C.  ****/

void  lcdBegin_cb(void)
  {
  delay(200);     // Time for LCD hardware to initialize
  // Now we pull both RS and R/W low to begin commands
  lcdExpanderWrite_cb(lcdBacklightval);   // reset expander & turn backlight off (Bit 8 =1)
  delay(1000);

  // Put the LCD into 4 bit mode, according to the HD44780 datasheet
  // figure 24, pg 46.  We start in 8bit mode, try to set 4 bit mode
  lcdWrite4bits_cb(0x03 << 4);
  delay(5); // wait min 4.1ms
  lcdWrite4bits_cb(0x03 << 4);  // second try
  delay(5); // wait min 4.1ms
  lcdWrite4bits_cb(0x03 << 4);  // third go!
  delay(1);

  lcdWrite4bits_cb(0x02 << 4); // Set to 4-bit interface

  lcdDisplayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
  lcdSend_cb((LCD_FUNCTIONSET | lcdDisplayfunction), 0);

  lcdDisplaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  lcdDisplay_cb();

  lcdClear_cb();    // clear it off

  lcdDisplaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  lcdSend_cb((LCD_ENTRYMODESET | lcdDisplaymode), 0); // Set entry mode

  backlight_cb();
  }

// Print all 16 characters of line 0 or 1 on lcd
void lcdPrintLine(int line)  {
  lcdSetCursor_cb(0, line);
  for(int i=0; i<16; i++)
     lcdWrite_cb(lcdData[line][i]);
  }

void lcdSetCursor_cb(uint8_t col, uint8_t row){
  if(row==1)
     lcdSend_cb((LCD_SETDDRAMADDR | (col + 0x40)), 0);
  else
     lcdSend_cb((LCD_SETDDRAMADDR | col), 0);
  }

void lcdClear_cb(){
    lcdSend_cb(LCD_CLEARDISPLAY, 0); // Clear display, set cursor to zero
    delay(2);
}

void lcdDisplay_cb() {
    lcdDisplaycontrol |= LCD_DISPLAYON;
    lcdSend_cb((LCD_DISPLAYCONTROL | lcdDisplaycontrol), 0);
}

void noBacklight_cb(void) {
    lcdBacklightval=LCD_NOBACKLIGHT;
    lcdExpanderWrite_cb(0);
}

void backlight_cb(void) {
    lcdBacklightval=LCD_BACKLIGHT;
    lcdExpanderWrite_cb(0);
}
// write either command (mode=0) or data (mode=1)
void lcdSend_cb(uint8_t value, uint8_t mode) {
    uint8_t highnib=value&0xf0;
    uint8_t lownib=(value<<4)&0xf0;
    lcdWrite4bits_cb((highnib)|mode);
    lcdWrite4bits_cb((lownib)|mode);
}

void lcdWrite4bits_cb(uint8_t value) {
    lcdExpanderWrite_cb(value);
    lcdPulseEnable_cb(value);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcdCreateChar_cb(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	//command(LCD_SETCGRAMADDR | (location << 3));
    lcdSend_cb(LCD_SETCGRAMADDR | (location << 3), 0);
	for (int i=0; i<8; i++) {
       lcdSend_cb(charmap[i], 1);
	}
}

// This is the entry point to I2C Wire
void lcdExpanderWrite_cb(uint8_t _data){
    Wire.beginTransmission(0x27);
    Wire.write((int)(_data) | lcdBacklightval);
    Wire.endTransmission();
}

void lcdPulseEnable_cb(uint8_t _data){
    lcdExpanderWrite_cb(_data | lcdEn);    // lcdEn high
    delayMicroseconds(1);        // enable pulse must be >450ns

    lcdExpanderWrite_cb(_data & ~lcdEn);    // lcdEn low
    delayMicroseconds(50);        // commands need > 37us to settle
}

void lcdWrite_cb(uint8_t value) {
    lcdSend_cb(value, lcdRs);
}
