#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include <drmLib.h>
#include <Wire.h>
#include <TM1637Display.h>

const char *ver="ClockDuino -> V7.0.0-20151214 ";
// A little tweeking to get to work with new clock module from ebay $1.59 from Seller: accecity2008
// alice1101983 also has lots of good stuff
// Works with both now, china module has memory also.
// shows date at top of minute now with V4
// Major rework of UI and add watchdog with V5
// Now shows temp at 45 sec for 3 sec with V6.3.0
// V6.3.5 Updated with V1.6.3 of the Dev Suite
// V6.3.6 made BAUD const long and getting ready for some things in library drm
// V6.3.7 now have problem with finding the TM1637 library, have to fix! is it an Arduino IDE issue?
// V6.4.0 don't sleep for 10 cycles when in the "set" process
// V7.0.0 modify to use the drmLib utility routines

// Display Module connection pins (Digital Pins)
#define DISP_CLK 4
#define DISP_DIO 2
// We use the IDE defined symbol LED_BUILTIN

volatile boolean wdt_int; // This is changed in the ISR for the watchdog

const long msec_repeat=500;
const byte print_every=2;
const int num_regs=19;
const int DS3231_addr=0x68; // DS3231 I2C address ChronoDot
const long BAUD=115200;

typedef struct parseTime {
  byte seconds;
  byte minutes;
  byte hours;
  byte dow;
  byte dom;
  byte month;
  byte year;
  byte csr;
  byte sr;
  int int_year;
  unsigned long lsec;
  long tempf;
};
struct parseTime time_struct[1];
    
/*
  See table Pg 55 datasheet for time-out variations:
  Watchdog preset scaler is set by 4 bits in the WDTCSR
        WDP3 WDP2 WDP1 WDP0
        VCC = 5.0V
        0 0 0 0 2K     (2048)   cycles   16 ms
        0 0 0 1 4K     (4096)   cycles   32 ms
        0 0 1 0 8K     (8192)   cycles   64 ms
        0 0 1 1 16K    (16384)  cycles   0.125 s
        0 1 0 0 32K    (32768)  cycles   0.25 s
        0 1 0 1 64K    (65536)  cycles   0.5 s
        0 1 1 0 128K   (131072) cycles   1.0 s
        0 1 1 1 256K   (262144) cycles   2.0 s
        1 0 0 0 512K   (524288) cycles   4.0 s
        1 0 0 1 1024K  (1048576)cycles   8.0 s
 */
const byte wdt_Setup = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0); // 0.25 sec
byte per_sec = 2; // will be updated automatically
byte cycle_down_cnt = 0;

unsigned long last_msec = 9999999; // initialize to weird value to assure quick first read
unsigned long last_sec=0;
boolean already=false;
byte bright = 0x0f;

byte sub_sec = 0;
byte prev_2dig_sec = 0;

TM1637Display display(DISP_CLK, DISP_DIO);

void setup()
{
  Serial.begin(BAUD);
  watchdogSetup();
  Wire.begin();
  DS3231_setup();
  pinMode(LED_BUILTIN, OUTPUT);
  drmStartPrint(ver);
  display.setBrightness(bright);
}
 
void loop()
{
  boolean t_wdt_int = wdt_int;
  char inbyte=NULL, tempbyte=NULL;
  byte read_by[num_regs];
  
  unsigned long new_msec = millis();
  boolean disp_update = (last_msec/msec_repeat != new_msec/msec_repeat);

  if(disp_update) last_msec = new_msec;

  if (Serial.available() > 0) {
    tempbyte = Serial.read();
    if(((tempbyte >= (byte) 'A') && tempbyte <= 'Z') | 
       ((tempbyte >= (byte) 'a') && tempbyte <= 'z')) inbyte = tempbyte; // ignore any other than alpha chars
  }

  if(inbyte != NULL || disp_update || t_wdt_int) {
    cycle_down_cnt = 10; // set counter for 10 cycles without sleep
    switch (inbyte) {
    case 'R': // Output all registers
      print_DS3231_registers(read_by); // print out registers, reset alarms and reguest temp
      read_Clock(read_by);
      print_DS3231_registers(read_by); // print out registers, reset alarms and reguest temp
      print_Time(read_by, new_msec);
      break;
    case 'W': // Write Clock values
      set_Time();
      read_Clock(read_by);
      print_Time(read_by, new_msec);
      break;
    case 'Y': // Increment Year
    case 'M': // Increment Month
    case 'D': // Increment Day
    case 'h': // Increment hour
    case 'm': // Increment minute
    case 's': // Increment second
      inc_Datetime(inbyte, read_by);
    break;
    case 'b': // cycle through brightness values
      display.setBrightness(0x0f & (bright++));
      Serial.println(0x0f & bright);
      break;
    default:;
      read_Clock(read_by); // read DS3231 registers
      break;
    }
    decode_Time(read_by);
    write_Disp();
    if(((time_struct->seconds) % print_every) == 0) {
      if (! already) {
        print_Time(read_by, new_msec);
        if(cycle_down_cnt > 0) cycle_down_cnt -= 1;
        already = true;
      }  
    }
    else {
      already = false;
    }
  }
  
  enterSleep(); // Sleep to conserve power
  if (t_wdt_int) { // reset the watchdog flag
    wdt_int = false;
  }
}

void inc_Datetime(byte inbyte, byte *read_by) {
  byte addr, mask, mod, offset=0;
  
  switch (inbyte) {
  case 'Y':
    addr = 6; mod = 100; mask = 0xff;
    break;
  case 'M': 
    addr = 5; mod = 12; mask = 0x1f; offset = 1;
    break;
  case 'D':
    addr = 4; mod = 31; mask = 0xff; offset = 1; // this will result in unreal dates, you have to set it right
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
  byte newbyte = (drmBcd2Dec(*(read_by+addr)) & mask);
//  Serial.print("newbyte1-> "); Serial.println(newbyte);
  newbyte = ((++newbyte - offset) % mod) + offset; // pre-increment the value with wraping 
//  Serial.print("newbyte2-> "); Serial.println(newbyte);
  newbyte = ((newbyte/10) << 4) | (newbyte % 10); // convert to BCD

  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write(addr); // address of BCD digits
  Wire.write(newbyte); // Write the incremented value
  Wire.endTransmission();
}

void clear_Alarms(byte RTC_status) {
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0x0f); // Status Register
  Wire.write((RTC_status & (byte) 0xFC)); // clear the alarm flags
  Wire.endTransmission();      
}  

void setconvert_Temp() { // I need to figure out what this does!
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

void decode_Time(byte *read_by) { // make sense out of the register valuse and put them in the global time structure time_struct
  // The below are all BCD encoded with some high control bits on some
  time_struct->seconds = read_by[0]; // get seconds
  time_struct->minutes = read_by[1]; // get minutes
  time_struct->hours = read_by[2];   // get hours
  time_struct->dow = read_by[3];   // get day of week (Mon = 0)
  time_struct->dom = read_by[4];   // get day of month
  time_struct->month = read_by[5];   // get month number (Jan = 1)
  time_struct->year = read_by[6];   // get year (last two digits)
  time_struct->csr = read_by[14];
  time_struct->sr = read_by[15];
  time_struct->seconds = drmBcd2Dec(time_struct->seconds);
  time_struct->minutes = drmBcd2Dec(time_struct->minutes);
  time_struct->hours = drmBcd2Dec(0x3F & time_struct->hours);
  time_struct->dow = drmBcd2Dec(time_struct->dow);
  time_struct->dom = drmBcd2Dec(time_struct->dom);
  time_struct->month = drmBcd2Dec(0x1F & time_struct->month);
  time_struct->year = drmBcd2Dec(time_struct->year);
  time_struct->int_year = 2000 + (100*((int) time_struct->month>32)) + (int) time_struct->year;
  time_struct->lsec = time_struct->seconds + 60*(time_struct->minutes + 60*(time_struct->hours + 24*time_struct->dom));
}

void print_Time(byte *read_by, long msecs) {
    drmPrtLead0(time_struct->int_year,4); Serial.print(F("/"));
    drmPrtLead0(time_struct->month,2); Serial.print(F("/"));
    drmPrtLead0(time_struct->dom,2); Serial.print(F(" "));
    
// Human readable date and time
    drmPrtLead0(time_struct->hours,2); Serial.print(F(":"));
    drmPrtLead0((long) time_struct->minutes, 2); Serial.print(F(":"));
    drmPrtLead0((long) time_struct->seconds, 2);    
    Serial.print(F(" PDT "));    
//    Serial.print(F(" PST "));

// Doug's date serial number to the second    
    drmPrtLead0((long) time_struct->int_year, 4);
    drmPrtLead0((long) time_struct->month, 2);
    drmPrtLead0((long) time_struct->dom, 2);
    Serial.print(F("_"));
    drmPrtLead0((long) (time_struct->hours & 0x3F), 2);
    drmPrtLead0((long) time_struct->minutes, 2);
    drmPrtLead0((long) time_struct->seconds, 2);

//  Temperature
    Serial.print(F(" T-> "));
    Serial.print(read_by[17]);
    Serial.print(F("."));
    drmPrtLead0((long) (read_by[18] >> 6)*25, 2);
    Serial.print(F("C"));
    time_struct->tempf = 3200 + ((read_by[17]*100 + (read_by[18] >> 6)*25)*9)/5;
    Serial.print(F("/"));
    Serial.print(time_struct->tempf/100);
    Serial.print(F("."));
    drmPrtLead0(time_struct->tempf%100, 2);
    Serial.print(F("F"));
    
// Delta seconds    
    Serial.print(F(" dS-> "));
    if ((time_struct->lsec - last_sec) < 30000) Serial.print(time_struct->lsec - last_sec); // Skip print when huge (startup)
    last_sec = time_struct->lsec;

// Status flags
    if((time_struct->sr & 0x04) != 0) Serial.print(F("  Bsy"));
    if((time_struct->sr & 0x01) != 0) Serial.print(F("  A0"));
    if((time_struct->sr & 0x02) != 0) Serial.print(F("  A1"));

    Serial.println("");
}

void set_Time() { // This is how I bootstrap the time on the DS3231, you must hardcode the setup time
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

void read_Clock(byte *read_by) {
  int i;
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, num_regs); // request read of all registers
  for (i=0;(i<num_regs && Wire.available());i++) { read_by[i] = Wire.read(); }
  if (i!=num_regs) { Serial.print(F("Read only ")); Serial.print(i); Serial.println(F(" bytes"));}
}

void print_DS3231_registers(byte *read_by) {
  setconvert_Temp();
  Serial.print(F("DigIn[5]-> ")); Serial.println(digitalRead(5));
  for (int i=0; i<num_regs; i++) {
    if(read_by[i]<16) Serial.print(F("0"));
    Serial.print(read_by[i], HEX); 
    Serial.print(F(" "));
  }
  Serial.println("");
  clear_Alarms(read_by[15]);
}

void write_Disp() {
  int num;
  byte colon=0x00;
  byte digits[4]={0,0,0,0};
  int num_hi = time_struct->hours;
  int num_lo = time_struct->minutes;
  boolean time=true;

/*
  Serial.print("Colon-> ");
  Serial.print(prev_2dig_sec);
  Serial.print(" - ");
  Serial.print(time_struct->seconds);
  Serial.print(" - ");
  Serial.print(per_sec/2);
  Serial.print(" - ");
  Serial.print(sub_sec);
  Serial.println("");
 */
  
  if(prev_2dig_sec != time_struct->seconds) {
    per_sec = max(per_sec, sub_sec);
    sub_sec=0;
    prev_2dig_sec = time_struct->seconds;
  }

  sub_sec++;
  if(sub_sec > (per_sec/2)) {
    colon=0x80; // prev_2dig_sec
  }
  
  if(time_struct->seconds >= 2 && time_struct->seconds <= 5) {
    num_hi = 20;
    num_lo = time_struct->year;
    colon = 0x00;
    time = false;
  }
  else if (time_struct->seconds >= 6 && time_struct->seconds <= 8) {
    num_hi = time_struct->month;
    num_lo = time_struct->dom;
    colon=0x80;    
    time = false;
  }
  else if (time_struct->seconds >= 46 && time_struct->seconds <= 48) {
    num_hi = (time_struct->tempf+50)/10000;
    num_lo = ((time_struct->tempf+50)%10000)/100;
    colon = 0x00;
    time = false;
  }
  digits[0]=display.encodeDigit(num_hi/10);
  digits[1]=display.encodeDigit(num_hi%10) | colon;
  if(! time && num_hi/10==0) {
    digits[0]=0x00;
    if(num_hi%10==0) digits[1]=0x00 | colon;
  }
  digits[2]=display.encodeDigit(num_lo/10);
  digits[3]=display.encodeDigit(num_lo%10);
  
  display.setSegments(digits);
}

void watchdogSetup(void)
{
  cli(); // disable interrupts
  wdt_reset();
/*
  WDTCSR configuration:
  WDIE = 1: Interrupt Enable
  WDE = 1 :Reset Enable
 */
// Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog time
  WDTCSR = wdt_Setup;
  sei(); // enable interrupts
}

ISR( WDT_vect ) {
  cli(); // disable interrupts
  wdt_reset();
// Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog time
  WDTCSR = wdt_Setup;
  wdt_int = true; // signal to main loop that the watchdog has fired
  sei(); // enable interrupts
}

void enterSleep(void)
{
  digitalWrite(LED_BUILTIN, LOW);
/*
 * The 5 different modes are:
 *     SLEEP_MODE_IDLE         -the least power savings
 *     SLEEP_MODE_ADC
 *     SLEEP_MODE_PWR_SAVE
 *     SLEEP_MODE_STANDBY
 *     SLEEP_MODE_PWR_DOWN     -the most power savings
 */
  if(cycle_down_cnt > 0) return; // skip the sleep if counting down from setting
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN, SLEEP_MODE_PWR_SAVE for lowest power consumption. */
  Serial.flush();
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  digitalWrite(LED_BUILTIN, HIGH);
  
  /* Re-enable the peripherals. */
  power_all_enable();
  Serial.begin(BAUD);
}
