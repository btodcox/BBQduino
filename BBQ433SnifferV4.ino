/*      Last edit: Dec 24, 2014
 *
 * BBQduino Maverick 732 BBQ Wireless Thermometer Sniffer v0.1x
 *     Also verified to work properly with Ivation Model #IVAWLTHERM BBQ Thermometer
 *
 *    (c) 2014 B. Tod Cox, John Cox
 *
 * BBQduino is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BBQduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
 *
 * Receives temperature data from a Maverick 732 (and clones) Wireless BBQ Thermometer
 * and outputs to an Adafruit HT1632 LED matrix as well as providing temps via a web server.
 *
 * Manchester decoding and interrupt handler based in part on code and ideas from  
 *         http://www.practicalarduino.com/projects/weather-station-receiver
 *         http://kayno.net/2010/01/15/arduino-weather-station-receiver-shield/
 *         https://forums.adafruit.com/viewtopic.php?f=8&t=25414&start=0
 *         http://wiki.openpicus.com/index.php?title=Wifi_bbq
 *     Web server & HTML/Canvas gauges sources:
 *         http://startingelectronics.com/tutorials/arduino/ethernet-shield-web-server-tutorial/  
 *         https://github.com/Mikhus/canv-gauge
 *
 *      Licenses of above works are include by reference.
 */

#include "HT1632n.h" //support files for adafruit HT1632 LED panels
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <EEPROM.h>
/* Note to self:  Pinout is set up for a REWORKED NCIDdisplay X4 */


//interface pins for LED panels
#define DATA A1
#define WR   A2
#define CS1  A3 //needs to be reworked from D4 for rev A PCB
//#define CS2  5
//#define CS3  6
//#define CS4  7

// size of buffer used to capture HTTP requests
#define REQ_BUF_SZ   50

// MAC address from Ethernet shield sticker under board
byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x00 };
IPAddress ip(10, 0, 0, 33); // IP address, may need to change depending on network
EthernetServer server(80);  // create a server at port 80
File webFile;               // the web page file on the SD card
char HTTP_req[REQ_BUF_SZ] = {
  0}; // buffered HTTP request stored as null terminated string
char req_index = 0;              // index into HTTP_req buffer


HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS1 );

#define INPUT_CAPTURE_IS_RISING_EDGE()    ((TCCR1B & _BV(ICES1)) != 0)
#define INPUT_CAPTURE_IS_FALLING_EDGE()   ((TCCR1B & _BV(ICES1)) == 0)
#define SET_INPUT_CAPTURE_RISING_EDGE()   (TCCR1B |=  _BV(ICES1))
#define SET_INPUT_CAPTURE_FALLING_EDGE()  (TCCR1B &= ~_BV(ICES1))

#define BBQ_RX_LED_ON()         ((PORTD &= ~(1<<PORTD6)))
#define BBQ_RX_LED_OFF()        ((PORTD |=  (1<<PORTD6)))

#define BBQ_RESET()             { short_count = packet_bit_pointer = 0; long_count = 0; BBQ_rx_state = RX_STATE_IDLE; current_bit = BIT_ZERO; }

#define TIMER_PERIOD_US             4  // Timer resolution is 4 micro seconds for 16MHz Uno
#define BBQ_PACKET_BIT_LENGTH   13*8  //13 bytes with 2 quaternary encoded nibbles each 

// pulse widths. short pulses ~500us, long pulses ~1000us. 50us tolerance
#define SHORT_PULSE_MIN_WIDTH       170/TIMER_PERIOD_US
#define SHORT_PULSE_MAX_WIDTH       300/TIMER_PERIOD_US
#define START_LONG_MIN_WIDTH        3700/TIMER_PERIOD_US  // Long low with short 5.2ms
#define START_LONG_MAX_WIDTH        5800/TIMER_PERIOD_US
#define LONG_PULSE_MIN_WIDTH        340/TIMER_PERIOD_US
#define LONG_PULSE_MAX_WIDTH        600/TIMER_PERIOD_US

// number of long/shorts in a row before the stream is treated as valid
#define SHORT_COUNT_SYNC_MIN        8

// states the receiver can be
#define RX_STATE_IDLE               0 // waiting for incoming stream
#define RX_STATE_READY              1 // "preamble" almost complete, next rising edge is start of data
#define RX_STATE_RECEIVING          2 // receiving valid stream
#define RX_STATE_PACKET_RECEIVED    3 // valid stream received

#define BIT_ZERO                    0
#define BIT_ONE                     1

#define DEBUG

// Type aliases for brevity in the actual code
typedef unsigned int       uint; //16bit
typedef signed int         sint; //16bit

volatile int ste=LOW;
volatile int new_data=0;
int pin13led = 13;

uint probe1, probe2, tmp_probe1, tmp_probe2;
uint32_t check_data;
uint16_t chksum_data, chksum_sent, chk_xor, chk_xor_expected=0;
boolean chk_xor_once;
uint captured_time;
uint previous_captured_time;
uint captured_period;
uint current_bit;
uint packet_bit_pointer;
uint short_count;
uint long_count;
uint BBQ_rx_state;

boolean previous_period_was_short = false;

// byte arrays used to store incoming weather data
byte BBQ_packet[(BBQ_PACKET_BIT_LENGTH/8)];
byte BBQ_packet_process[(BBQ_PACKET_BIT_LENGTH/8)];
byte probe1_array[6], probe2_array[6]; //only need the 6 nibbles with actual probe temp data
byte last_BBQ_packet[(BBQ_PACKET_BIT_LENGTH/8)];

/* Overflow interrupt vector */
ISR(TIMER1_OVF_vect){                 // here if no input pulse detected
}

/* ICR interrupt vector */
ISR(TIMER1_CAPT_vect){
  int i; //"temp" variable
  // Immediately grab the current capture time in case it triggers again and
  // overwrites ICR1 with an unexpected new value
  captured_time = ICR1;

  //immediately grab the current capture polarity and reverse it to catch all the subsequent high and low periods coming in
  if(INPUT_CAPTURE_IS_RISING_EDGE()) {
    SET_INPUT_CAPTURE_FALLING_EDGE();      //previous period was low and just transitioned high   
  } 
  else {
    SET_INPUT_CAPTURE_RISING_EDGE();       //previous period was high and transitioned low    
  }

  // calculate the current period just measured, to accompany the polarity now stored
  captured_period = (captured_time - previous_captured_time);

  // Analyse the incoming data stream. If idle, we need to detect the start of an incoming weather packet.
  // Incoming packet starts with several short pulses (over 100 short pulses) before a long pulse to signify 
  // the start of the data.

  if(BBQ_rx_state == RX_STATE_IDLE) {

    if(( (captured_period >= SHORT_PULSE_MIN_WIDTH) && (captured_period <= SHORT_PULSE_MAX_WIDTH)  )) { 
      // short pulse, continue counting short pulses
      short_count++;
      if( short_count != long_count) { //should only have long (~5 ms) low followed by (~250 usec) high; if not, clear count
        short_count = 0;
        long_count = 0;
      }
      if(short_count == SHORT_COUNT_SYNC_MIN) { //8 long/short start pulses in row; packet with info is about to start!
        BBQ_rx_state = RX_STATE_READY;
#ifdef DEBUG
        digitalWrite(7,LOW); //helpful for tracking RX_STATE with logic analyzer
#endif
      } 
    } 
    else if(((captured_period >= START_LONG_MIN_WIDTH) && (captured_period <= START_LONG_MAX_WIDTH) )) { 
      // long pulse. if there has been enough short pulses beforehand, we have a valid bit stream, else reset and start again
      long_count++;
      //ste = !ste;
      // digitalWrite(pin13led, ste);

    } 
    else {
      BBQ_RESET();
    } 
  } 
  else if (BBQ_rx_state == RX_STATE_READY) {
    digitalWrite(pin13led,0);
    //ste =!ste;
    // this rising edge (assuming a ~4.8msec low) is start of data
    if (((captured_period >= START_LONG_MIN_WIDTH) && (captured_period <= START_LONG_MAX_WIDTH))){
      // start of data packet
      // long pulse
      // swap the currrent_bit
      current_bit = !current_bit;
      BBQ_packet[packet_bit_pointer >> 3] |=  (0x80 >> (packet_bit_pointer&0x07)); //store the bit
      packet_bit_pointer++;
      previous_period_was_short = false;
      BBQ_rx_state = RX_STATE_RECEIVING;
    }
    else{ 
      BBQ_RESET(); 
    }
  } 
  else if(BBQ_rx_state == RX_STATE_RECEIVING) {
    // incoming pulses are a valid bit stream, manchester encoded. starting with a zero bit, the next bit will be the same as the 
    // previous bit if there are two short pulses, or the bit will swap if the pulse is long
    if(((captured_period >= SHORT_PULSE_MIN_WIDTH) && (captured_period <= SHORT_PULSE_MAX_WIDTH))) {  
      // short pulse
      if(previous_period_was_short) { 
        // previous bit was short, add the current_bit value to the stream and continue to next incoming bit
        if(current_bit == BIT_ONE) {
          BBQ_packet[packet_bit_pointer >> 3] |=  (0x80 >> (packet_bit_pointer&0x07));
        } 
        else if (current_bit == BIT_ZERO) {
          BBQ_packet[packet_bit_pointer >> 3] &= ~(0x80 >> (packet_bit_pointer&0x07));
        }

        packet_bit_pointer++;

        previous_period_was_short = false;
      } 
      else {
        // previous bit was long, remember that and continue to next incoming bit
        previous_period_was_short = true;
      }
    } 
    else if(((captured_period >= LONG_PULSE_MIN_WIDTH) && (captured_period <= LONG_PULSE_MAX_WIDTH))) { 
      // long pulse
      // swap the current_bit
      //     if (previous_period_was_short){ //cannot have a long pulse if previous_period was short
      //       BBQ_RESET();
      //    }
      current_bit = !current_bit;

      // add current_bit value to the stream and continue to next incoming bit
      if(current_bit == BIT_ONE) {
        BBQ_packet[packet_bit_pointer >> 3] |=  (0x80 >> (packet_bit_pointer&0x07));
      } 
      else if (current_bit == BIT_ZERO) {
        BBQ_packet[packet_bit_pointer >> 3] &= ~(0x80 >> (packet_bit_pointer&0x07));
      }
      packet_bit_pointer++;
    }


  }
  // check to see if a full packet has been received
  if(packet_bit_pointer >= BBQ_PACKET_BIT_LENGTH) {

    // full packet received, switch state to RX_STATE_PACKET_RECEIVED
    for (i=0; i<=13; i++){
      BBQ_packet_process[i]=BBQ_packet[i];
    }
    //BBQ_rx_state = RX_STATE_PACKET_RECEIVED;
    new_data = 1; //signal to main loop that new packet is available
    digitalWrite(7,HIGH);
    digitalWrite(13,HIGH);
    BBQ_RESET();
  }
  // save the current capture data as previous so it can be used for period calculation again next time around
  previous_captured_time = captured_time;
  //       if (packet_bit_pointer >= BBQ_PACKET_BIT_LENGTH){
  //        digitalWrite(7,HIGH);
  //     }
}

void setup() {
  Serial.begin(115200);
  chk_xor_once = false; //flag on whether we have set the expected output of XOR of checksome from transmitter
  // initialize SD card
  Serial.println(F("Initializing SD card..."));
  if (!SD.begin(4)) {
    Serial.println(F("ERROR - SD card initialization failed!"));
    return;    // init failed
  }
  Serial.println(F("SUCCESS - SD card initialized."));
  // check for index.htm file
  if (!SD.exists("index.htm")) {
    Serial.println(F("ERROR - Can't find index.htm file!"));
    return;  // can't find index file
  }
  Serial.println(F("SUCCESS - Found index.htm file."));

  Ethernet.begin(mac, ip);  // initialize Ethernet device
  server.begin();           // start to listen for clients


  //setup LED panel
  matrix.begin(HT1632_COMMON_16NMOS);  
  matrix.fillScreen();
  delay(1000);
  matrix.clearScreen();
  matrix.setTextColor(1);
  matrix.setTextSize(1);
  
  matrix.clearBuffer();
  matrix.setCursor(0,0);
  matrix.print(F("Sync"));
  matrix.setCursor(0,8);
  matrix.print(F("Sndr"));
  matrix.writeScreen();	
  
  // read last checksum value out of EEPROM to eliminate need to resync if you reset or powercycle arduino
  chk_xor_expected = EEPROM.read(0);
  chk_xor_expected |= EEPROM.read(1) << 8;
 
#ifdef DEBUG
  Serial.print("Last checksum_xor value: ");
  Serial.println(chk_xor_expected, HEX);
#endif

  pinMode(pin13led, OUTPUT);
  digitalWrite(pin13led, HIGH);

  //prep for logic analyzer capture
  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);

  // setup Input Capture Unit interrupt (ICP1) that allows precise timing of pulses
  // which, for this program, allows us to decode the 2000baud temp data from the BBQ transmitter
  DDRB = 0x2F;   // B00101111
  DDRB  &= ~(1<<DDB0);    // PBO(ICP1) input
  PORTB &= ~(1<<PORTB0);  // ensure pullup resistor is also disabled
  DDRD  |=  B11000000;    // (1<<PORTD6);   //DDRD  |=  (1<<PORTD7); (example of B prefix)

  //---------------------------------------------------------------------------------------------
  //ICNC1: Input Capture Noise Canceler         On, 4 successive equal ICP1 samples required for trigger (4*4uS = 16uS delayed)
  //ICES1: Input Capture Edge Select            1 = rising edge to begin with, input capture will change as required
  //CS12,CS11,CS10   TCNT1 Prescaler set to 0,1,1 see table and notes above
  TCCR1A = B00000000;   //Normal mode of operation, TOP = 0xFFFF, TOV1 Flag Set on MAX
  //This is supposed to come out of reset as 0x00, but something changed it, I had to zero it again here to make the TOP truly 0xFFFF
  TCCR1B = ( _BV(ICNC1) | _BV(CS11) | _BV(CS10) );
  SET_INPUT_CAPTURE_RISING_EDGE();
  //Timer1 Input Capture Interrupt Enable, Overflow Interrupt Enable  
  TIMSK1 = ( _BV(ICIE1) | _BV(TOIE1) );

  BBQ_RESET();
  Serial.println(F("BTC BBQ Sniffer  v0.15"));
  Serial.println(F("Ready to receive temp data"));
}


// make the quarternary convertion
byte quart(byte param)
{
  param &= 0x0F;
  if (param==0x05) return(0);
  if (param==0x06) return(1);
  if (param==0x09) return(2);
  if (param==0x0A) return(3);
}

double get_temp(byte select)
{
  if (select==1) return((double)probe1);
  if (select==2) return((double)probe2);
  //	if (select==3) return((double)DS1820_temp);

  return(0);	// just for safety
}

uint16_t shiftreg(uint16_t currentValue) {
  uint8_t msb = (currentValue >> 15) & 1;
  currentValue <<= 1;
  if (msb == 1) {
    // Toggle pattern for feedback bits
    // Toggle, if MSB is 1
    currentValue ^= 0x1021;
  }
  return currentValue;
}

//data = binary representation of nibbles 6 - 17
//e.g. xxxx:xxxx:xxxx:0010:1000:1010:0110:0101:0101:xxxx:xxxx:xxxx:xxxx

//  -> uint32_t data = 0x28a655
uint16_t calculate_checksum(uint32_t data) {
  uint16_t mask = 0x3331; //initial value of linear feedback shift register
  uint16_t csum = 0x0;
  int i = 0;
  for(i = 0; i < 24; ++i) {
    if((data >> i) & 0x01) {
      //data bit at current position is "1"
      //do XOR with mask
      csum ^= mask; 
    }
    mask = shiftreg(mask);
  }
  return csum;
}

void webserve(){
  EthernetClient client = server.available();  // try to get client

  if (client) {  // got client?
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {   // client data available to read
        char c = client.read(); // read 1 byte (character) from client
        // buffer first part of HTTP request in HTTP_req array (string)
        // leave last element in array as 0 to null terminate string (REQ_BUF_SZ - 1)
        if (req_index < (REQ_BUF_SZ - 1)) {
          HTTP_req[req_index] = c;          // save HTTP request character
          req_index++;
        }
        // last line of client request is blank and ends with \n
        // respond to client only after last line received
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          // remainder of header follows below, depending on if
          // web page or XML page is requested
          // Ajax request - send XML file
          if (StrContains(HTTP_req, "ajax_inputs")) {
            // send rest of HTTP header
            client.println(F("Content-Type: text/xml"));
            client.println(F("Connection: keep-alive"));
            client.println();
            // send XML file containing input states
            XML_response(client);
          }
          else {  // web page request
            // send rest of HTTP header
            Serial.println(F("index.htm request"));
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            // send web page
            webFile = SD.open("index.htm");        // open web page file
            if (webFile) {
              while(webFile.available()) {
                client.write(webFile.read()); // send web page to client
              }
              webFile.close();
            }
          }
          // display received HTTP request on serial port
          Serial.println(HTTP_req);
          // reset buffer index and all buffer elements to 0
          req_index = 0;
          StrClear(HTTP_req, REQ_BUF_SZ);
          break;
        }
        // every line of text received from the client ends with \r\n
        if (c == '\n') {
          // last character on line of received text
          // starting new line with next character read
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // a text character was received from client
          currentLineIsBlank = false;
        }
      } // end if (client.available())
    } // end while (client.connected())
    delay(1);      // give the web browser time to receive the data
    client.stop(); // close the connection
  } // end if (client)
}

// send the XML file containing analog value
void XML_response(EthernetClient cl)
{
  cl.print(F("<?xml version = \"1.0\" ?>"));
  cl.print(F("<inputs>"));
  cl.print(F("<temp>"));
  cl.print(probe1,DEC);
  cl.print(F("</temp>"));
  cl.print(F("<temp>"));
  cl.print(probe2,DEC);
  cl.print(F("</temp>"));
  cl.print(F("</inputs>"));
}

// sets every element of str to 0 (clears array)
void StrClear(char *str, char length)
{
  for (int i = 0; i < length; i++) {
    str[i] = 0;
  }
}

// searches for the string sfind in the string str
// returns 1 if string found
// returns 0 if string not found
char StrContains(char *str, char *sfind)
{
  char found = 0;
  char index = 0;
  char len;

  len = strlen(str);

  if (strlen(sfind) > len) {
    return 0;
  }
  while (index < len) {
    if (str[index] == sfind[found]) {
      found++;
      if (strlen(sfind) == found) {
        return 1;
      }
    }
    else {
      found = 0;
    }
    index++;
  }

  return 0;
}

void loop() {
  int i; //looping dummy
  webserve();
  if(new_data){
    new_data = 0; //clear the flag that new data is available in BBQ_packet_process array
        matrix.setPixel(23,0);  //flash led in upper right corner to indicate reception of packet (valid or invalid)
        matrix.writeScreen();
#ifdef DEBUG 
    for( i = 0; i < ((BBQ_PACKET_BIT_LENGTH/8)); i++) {
      Serial.print(BBQ_packet_process[i], HEX);
      Serial.print(" ");
    }  
    Serial.println();
#endif
    if ( (BBQ_packet_process[0] == 0xAA) && 
         (BBQ_packet_process[1] == 0x99) &&
         (BBQ_packet_process[2] == 0x95)  &&
       ( (BBQ_packet_process[3] == 0x59) || //regular data packet
         (BBQ_packet_process[3] == 0x6A) )  //update transmitter chk_xor_expected--still contains temp info!!!
    )
    {
      tmp_probe2=tmp_probe1=0;

      // convert temp packet from quaternary encoding
      probe2_array[0]= quart(BBQ_packet_process[8] & 0x0F);
      probe2_array[1]= quart(BBQ_packet_process[8] >> 4);
      probe2_array[2]= quart(BBQ_packet_process[7] & 0x0F);
      probe2_array[3]= quart(BBQ_packet_process[7] >> 4);
      probe2_array[4]= quart(BBQ_packet_process[6] & 0x0F);

      probe1_array[0]= quart(BBQ_packet_process[6] >> 4);
      probe1_array[1]= quart(BBQ_packet_process[5] & 0x0F);
      probe1_array[2]= quart(BBQ_packet_process[5] >> 4);
      probe1_array[3]= quart(BBQ_packet_process[4] & 0x0F);
      probe1_array[4]= quart(BBQ_packet_process[4] >> 4);

      for (i=0;i<=4;i++){
        tmp_probe2 += probe2_array[i] * (1<<(2*i));
      }

      for (i=0;i<=4;i++){
        tmp_probe1 += probe1_array[i] * (1<<(2*i));
      }

      //calc checksum and XOR with sent checksum to see if we got good data from correct transmitter
      //checksum calculation needs nibbles 6-17; see adafruit link for info.
      check_data = (uint32_t) quart(BBQ_packet_process[3] >> 4) << 22;
      check_data |= (uint32_t) quart(BBQ_packet_process[3]  & 0x0F) << 20;
      check_data |= (uint32_t) tmp_probe1 << 10;
      check_data |= (uint32_t) tmp_probe2;

      chksum_data = calculate_checksum(check_data);

      // nibbles 18-21 have checksum info from sender
      // convert sent checksum nibbles from quaternary encoding
      chksum_sent =  (uint16_t) quart(BBQ_packet_process[9] >> 4)     << 14;
      chksum_sent |= (uint16_t) quart(BBQ_packet_process[9]  & 0x0F)  << 12;
      chksum_sent |= (uint16_t) quart(BBQ_packet_process[10] >> 4)    << 10;
      chksum_sent |= (uint16_t) quart(BBQ_packet_process[10]  & 0x0F) << 8;
      chksum_sent |= (uint16_t) quart(BBQ_packet_process[11] >> 4)    << 6;
      chksum_sent |= (uint16_t) quart(BBQ_packet_process[11]  & 0x0F) << 4;
      chksum_sent |= (uint16_t) quart(BBQ_packet_process[12] >> 4)    << 2;
      chksum_sent |= (uint16_t) quart(BBQ_packet_process[12]  & 0x0F);

      // if packet is valid and from correct transmitter, chk_xor is constant
      // chk_xor will be different for each transmitter and will only change when
      //    a) transmitter is powered off/on
      //    b) sync/reset button is pressed on transmitter
      // Maverick wireless BBQ thermometers only allow the receiver to update the
      // transmitter chk_xor ONCE.  any new 6A packets are ignored by receiver until 
      // receiver is power cylced or reset.

      chk_xor = chksum_data ^ chksum_sent; 
#ifdef DEBUG                
      Serial.print("chx: ");
      Serial.println(chk_xor, HEX);
#endif          
      //check if we need to update chk_xor_expected
      if ( (BBQ_packet_process[3] == 0x6A) &&
        (chk_xor_once == false) ) // only allow the chk_xor_expected to be updated ONCE
      {  
        chk_xor_expected = chk_xor;
        chk_xor_once = true;
        //store new value in EEPROM so can survive reset or powercycle without resyncing with transmitter
        EEPROM.write(0, (byte) chk_xor_expected);
        EEPROM.write(1, ( (byte) (chk_xor_expected >> 8)) );
      }


      // finish up probe temp calculations to yield celcius temps
      // if the chk_xor is good for current packet
      // and update temps/display if all is good
      if (chk_xor == chk_xor_expected)
      {	
        chk_xor_once = true;  // could have a valid chk_xor_expected stored in EEPROM, if so, prevent resync without reset/powercyle
        matrix.setPixel(23,15);  //flash led in lower right corner to indicate reception of valid packet
        matrix.writeScreen();	
        if (tmp_probe1 != 0){ //check for unplugged temp probe
            probe1 = tmp_probe1-532; 
            probe1 = (probe1 * 18 + 5)/10 + 32; //convert to fahrenheit using fast integer math 
        } 
        else
        {
          probe1 = 0;  //probe temp of 0 indicates unplugged temp probe on transmitter
        }
        if (tmp_probe2 != 0){ //check for unplugged temp probe
          probe2 = tmp_probe2-532; 
          probe2 = (probe2 * 18 + 5)/10 + 32; //convert to fahrenheit using fast integer math 
        } 
        else
        {
          probe2 = 0; //probe temp of 0 indicates unplugged temp probe on transmitter
        }
        
        Serial.print("Food: ");
        Serial.println(probe1, DEC);
        Serial.print("Pit : ");
        Serial.println(probe2, DEC);

        //update display & we are "done" for now!
        matrix.clearBuffer();  //will also clear led that was set to indicate valid packet received
        matrix.setCursor(0,0);
        matrix.print("F");
        if (probe1 == 0) { // no temp probe attached 
           matrix.print("---"); 
        } 
        else //valid temp data
        {
          if (probe1 < 100) matrix.print(" "); 
          matrix.print(probe1,DEC);
        }
        matrix.setCursor(0,8);
        matrix.print("B");
        if (probe2 == 0)
        {
          matrix.print("---");
        }
        else // valid temp data
        {
          if (probe2 < 100) matrix.print(" ");
          matrix.print(probe2,DEC);
        }
        matrix.writeScreen();	//update LED display
      } else // clear pixel received pixel
      {
        matrix.clrPixel(23,0);
        matrix.writeScreen();
      }	
      	
    }
  }     
} 



