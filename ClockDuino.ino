#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <TM1637Display.h>

// Display Module connection pins (Digital Pins)
#define DISP_CLK 4
#define DISP_DIO 2

const char *version="ChronoDot_20141027 -> V5.1.0-20141126 ";
// A little tweeking to get to work with new clock module from ebay $1.59 from Seller: accecity2008 
// Works with both now, china module has memory also.
// shows date at top of minute now with V4
// Major rework of UI and add watchdog with V5
//
const long msec_repeat=500;
const int num_regs = 19;
const int DS3231_addr = 0x68; // DS3231 I2C address ChronoDot
// const int DS3231_addr = 0x57; // DS3231 I2C address China Board
unsigned long last_msec = 9999999; // initialize to weird value to assure quick first read
unsigned long last_sec=0;
byte bright = 0x0f;

TM1637Display display(DISP_CLK, DISP_DIO);

void setup()
{
  Serial.begin(115200);
  watchdogSetup();
  Wire.begin();
  DS3231_setup();
  drm_start_print();
  display.setBrightness(bright);
}
 
void loop()
{
  char inbyte=NULL, tempbyte=NULL;
  byte read_by[num_regs];
  
  unsigned long new_msec = millis();
  wdt_reset();
  LED_Blink(11, last_msec, new_msec);  // Flash an LED on PWM pin 11
  
  boolean next_sec = (last_msec/msec_repeat != new_msec/msec_repeat);
  if(next_sec) last_msec=new_msec;

  if (Serial.available() > 0) {
    tempbyte = Serial.read();
    if(((tempbyte >= (byte) 'A') && tempbyte <= 'Z') | 
       ((tempbyte >= (byte) 'a') && tempbyte <= 'z')) inbyte = tempbyte; // ignore any other than alpha chars
  }

  if(inbyte != NULL || next_sec) {
    read_clock(read_by); // read DS3231 registers

    switch (inbyte) {
    case 'R': // Output all registers
      print_DS3231_registers(read_by); // print out registers, reset alarms and reguest temp
      read_clock(read_by);
      break;
    case 'W': // Write Clock values
      set_time();
      read_clock(read_by);
      break;
    case 'Y':
    case 'M':
    case 'D':
    case 'h':
    case 'm':
    case 's':
      inc_datetime(inbyte, read_by);
    break;
    case 'b':
      display.setBrightness(0x0f & (bright++));
      Serial.println(0x0f & bright);
      break;
    default:;
    }
    print_time(read_by, new_msec);
  }
}

void inc_datetime(byte inbyte, byte *read_by) {
  byte addr, mask, mod;
  
  switch (inbyte) {
  case 'Y':
    addr = 6; mod = 100; mask = 0xff;
    break;
  case 'M': 
    addr = 5; mod = 12; mask = 0x1f;
    break;
  case 'D':
    addr = 4; mod = 31; mask = 0xff;
    break;
  case 'h':
    addr = 2; mod = 24; mask = 0x3f;
    break;
  case 'm':
    addr = 1; mod = 60; mask = 0xff;
    break;
  case 's':
    addr = 0; mod = 60; mask = 0xff;
    break;
  }
  byte newbyte = (1+(bcd2dec_byte(*(read_by+addr)) & mask)) % mod;
  (*(read_by+addr) + 1) % mod; // increment the value with wraping 
  newbyte = ((newbyte/10) << 4) | (newbyte % 10); // convert to BCD

  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write(addr); // address of BCD digits
  Wire.write(newbyte); // Write the incremented value
  Wire.endTransmission();
}

unsigned short drm_serialno() {
  return(EEPROM.read(5) << 8 | EEPROM.read(6)); // combine two bytes into in serial number (drm specific)
}

byte bcd2dec_byte(byte in_byte) {
  return (((in_byte & 0b11110000)>>4)*10 + (in_byte & 0b00001111));
}

void s_prt_lead0(long in, int places) {
  if(places>10 || places<2) return;
  in = abs(in); // only for positive numbers
  if(in < 1000000000) in = in + 1000000000; // extend smaller numbers
  char out_str[11];
  sprintf(out_str, "%ld", in);
  Serial.print((out_str+(10-places)));
}

void clear_alarms(byte RTC_status) {
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0x0f); // Status Register
  Wire.write((RTC_status & (byte) 0xFC)); // clear the alarm flags
  Wire.endTransmission();      
}  

void convert_temp() {
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0x0e); // start at register 0x0F Status Register
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, 1); // Read one byte only
  byte RTC_cr = Wire.read();

  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0x0e); // Status Register
  Wire.write((RTC_cr | (byte) 0x20)); // clear the alarm flags
  Wire.endTransmission();      
}

void DS3231_setup() {
  // clear /EOSC bit
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x0E); // start at the CSR
  Wire.write(0b00000110); // write register bitmap, bit 7 is /EOSC
                          // bit 2 - 0 are interupt/square wave/alarm enables, 
                          // #2 choses between sqwv and alarms
                          // #1 turns on alarm 2
                          // #0 turns on alarm 1
  Wire.write(0b00000000); // Clear the flags in the Status register
  Wire.endTransmission();

// Set up the Alarms
  Wire.beginTransmission(DS3231_addr); // address DS3231
  Wire.write(0x07); // select register -- Alarm registers
  Wire.write(0b10000000);
  Wire.write(0b10000000);
  Wire.write(0b10000000);
  Wire.write(0b10000000);
  Wire.write(0b10000000);
  Wire.write(0b10000000);
  Wire.write(0b10000000); // Alarm 1 every second Alarm 2 every minute
  Wire.endTransmission();

// Set up the Aging offset
  Wire.beginTransmission(DS3231_addr); // address DS3231
  Wire.write(0x10); // Aging offset select register -- Alarm registers
  Wire.write(0b00000000); // No Aging offset
  Wire.endTransmission();
}

void drm_start_print() {
  Serial.print(version); Serial.print(F(" SN#"));
  Serial.println(drm_serialno());
  Serial.print(F("Compiled-> "));
  Serial.print(F(__DATE__)); 
  Serial.print(F(" "));
  Serial.println(F(__TIME__));
}

void print_time(byte *read_by, long msecs) {
    // The below are all BCD encoded with some high control bits on some
    int seconds = read_by[0]; // get seconds
    int minutes = read_by[1]; // get minutes
    int hours = read_by[2];   // get hours
    int dow = read_by[3];   // get day of week (Mon = 0)
    int dom = read_by[4];   // get day of month
    int month = read_by[5];   // get month number (Jan = 1)
    int year = read_by[6];   // get year (last two digits)
    byte csr = read_by[14];
    byte sr = read_by[15];
    seconds = bcd2dec_byte(seconds);
    minutes = bcd2dec_byte(minutes);
    hours = bcd2dec_byte(0x3F & hours);
    dow = bcd2dec_byte(dow);
    dom = bcd2dec_byte(dom);
    month = bcd2dec_byte(0x1F & month);
    year = bcd2dec_byte(year);
    int int_year = 2000 + (100*(month>32)) + (long) year;
    unsigned long lsec = seconds + 60*(minutes + 60*(hours + 24*dom));
    
    write_Disp(year, month, dom, hours, minutes, seconds, msecs);

    s_prt_lead0(int_year,4); Serial.print(F("/"));
    s_prt_lead0(month,2); Serial.print(F("/"));
    s_prt_lead0(dom,2); Serial.print(F(" "));
    
// Human readable date and time
    s_prt_lead0(hours,2); Serial.print(F(":"));
    s_prt_lead0((long) minutes, 2); Serial.print(F(":"));
    s_prt_lead0((long) seconds, 2);    
    Serial.print(F(" PST "));

// Doug's date serial number to the second    
    s_prt_lead0((long) int_year, 4);
    s_prt_lead0((long) month, 2);
    s_prt_lead0((long) dom, 2);
    Serial.print(F("_"));
    s_prt_lead0((long) (hours & 0x3F), 2);
    s_prt_lead0((long) minutes, 2);
    s_prt_lead0((long) seconds, 2);

//  Temperature
    Serial.print(F(" T-> "));
    Serial.print(read_by[17]);
    Serial.print(F("."));
    s_prt_lead0((long) (read_by[18] >> 6)*25, 2);
    Serial.print(F("C"));
    long Ftemp = 3200 + ((read_by[17]*100 + (read_by[18] >> 6)*25)*9)/5;
    Serial.print(F("/"));
    Serial.print(Ftemp/100);
    Serial.print(F("."));
    s_prt_lead0(Ftemp%100, 2);
    Serial.print(F("F"));
    
// Delta seconds    
    Serial.print(F(" dS-> "));
    Serial.print(lsec - last_sec); 
    last_sec = lsec;

// Status flags
    if((sr & 0x04) != 0) Serial.print(F("  Bsy"));
    if((sr & 0x01) != 0) Serial.print(F("  A0"));
    if((sr & 0x02) != 0) Serial.print(F("  A1"));

    Serial.println("");
}

void set_time() { // This is how I bootstrap the time on the DS3231, you must hardcode the setup time
  Wire.beginTransmission(DS3231_addr);
//  Set Time
  Wire.write((byte) 0x00); // start at register 0
  Wire.write((byte) 0x00); // Seconds
  Wire.write((byte) 0x49); // Minutes
  Wire.write((byte) 0x19); // Hour register
//  Set Date
//  Wire.write((byte) 0x03); // start at register 3
//  Wire.write((byte) 0x03); // Day of Week
//  Wire.write((byte) 0x12); // Day of Month
//  Wire.write((byte) 0x11); // Month
//  Wire.write((byte) 0x14); // Year
  Wire.endTransmission();
}

void read_clock(byte *read_by) {
  int i;
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, num_regs); // request read of all registers
  for (i=0;(i<num_regs && Wire.available());i++) { read_by[i] = Wire.read(); }
  if (i!=num_regs) { Serial.print(F("Read only ")); Serial.print(i); Serial.println(F(" bytes"));}
}

void print_DS3231_registers(byte *read_by) {
  convert_temp();
  Serial.print(F("DigIn[5]-> ")); Serial.println(digitalRead(5));
  for (int i=0; i<num_regs; i++) {
    if(read_by[i]<16) Serial.print(F("0"));
    Serial.print(read_by[i], HEX); 
    Serial.print(F(" "));
  }
  Serial.println("");
  clear_alarms(read_by[15]);
}

boolean LED_Blink(int pin, unsigned long last_msec, unsigned long new_msec) {
  // Flash an LED on PWM pin 11
  const unsigned long period=2000;
  int intensity=254;
  if (new_msec % period < period/2) intensity = 0;
  analogWrite(pin, intensity);
  return(intensity);
}

void write_Disp(int year,int month,int dom, int hours, int minutes, int seconds, long msecs) {
  int num;
  byte colon=0x00;
  byte digits[4]={0,0,0,0};
  int num_hi = hours;
  int num_lo = minutes;

  if((msecs % 1000) < (int) 500) colon=0x80;
  if(seconds >= 2 && seconds <= 5) {
    num_hi = 20;
    num_lo = year;
    colon = 0;
  }
  else if (seconds >= 6 && seconds <= 8) {
    num_hi = month;
    num_lo = dom;
    colon=0x80;    
  }
  digits[0]=display.encodeDigit(num_hi/10);
  digits[1]=display.encodeDigit(num_hi - 10*(num_hi/10)) | colon;
  digits[2]=display.encodeDigit(num_lo/10);
  digits[3]=display.encodeDigit(num_lo - 10*(num_lo/10));
  
  display.setSegments(digits);
}

void watchdogSetup(void)
{
  cli();
  wdt_reset();
/*
  WDTCSR configuration:
  WDIE = 1: Interrupt Enable
  WDE = 1 :Reset Enable
  See table in datasheet for time-out variations:
 */
// Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog time and results:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  sei();
  delayMicroseconds(100);
}
