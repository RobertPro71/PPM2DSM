//Beschreibung der Signale
//Werte eine orginal DX4
//
// Kanal     1         2                    3                   4
// Maximal 0x0353    0x0753 => 0x0353    0x0B53 => 0x0353    0x0F53 => 0x0353     =>   851  dec 
// Mitte   0x01BB    0x05F4 => 0x01FA    0x09FA => 0x01FA    0x0DFA => 0x01FA     =>   506  dec
// Minimal 0x00AA    0x04AA => 0x00AA    0x08AA => 0x00AA    0x0CAA => 0x00AA     =>   170  dec  
//
//  Tool zur Berechnung http://www.arndt-bruenner.de/mathe/9/geradedurchzweipunkte.htm
//
//Multiplex Cockpit MM
//====================
//
//Gemessener Impuls:
//
//         AVR gemessen       Scope gemessen   Logic gemessen
//Maximal  0x76E  = 1,902  =  1,92ms         =  1,90ms
//Mitte    0x5DD  = 1,504  =  ms             =  1,50ms
//Minimal  0x44D  = 1,101  =  1,16ms         =  1,12ms
//
//Errechnet 
//m = 0,85018
//b = -766
//
//Graupner MC16/20
//================
//
//Maximal  0x754  = 1,876  =  1,90ms         =  ms
//Mitte    0x5D0  = 1,488  =  1,52ms         =  ms
//Minimal  0x444  = 1,092  =  1,04ms         =  ms
//
//Errechnet 
//m = 0,89951
//b = -832

//Laut Internet Pulslänge 1,0 - 2,0ms wobei 1,5ms Mittelstellung ist.

//Securety Code
//String         Copyright Robert Proeschild
//String as HEX  0x43 6F 70 79 72 69 67 68 74 20 52 6F 62 65 72 74 20 50 72 6F 65 73 63 68 69 6C 64
//Diff as DEZ      43       
#include "Arduino.h"
#include <avr/interrupt.h>
#include <EEPROM.h>

#define VERSION 6

#define _DEBUG

//Module
#define FULLRANGE
#define _SHORTRANGE

//Transmitter
#define _GRAUPNER_IFS_MC
#define _GRAUPNER_HOTT
#define FUTABA
#define _MULTIPLEX
//Hardware
#define _SINGLE_LED
#define _DEBUG_PIN

#define _WRITE_EEPROM

typedef enum {
    NULL_ST = -1, NOT_SYNCHED, ACQUIRING, READY, FAILSAFE
} State_t;

typedef enum {
  LED_PPM_OK,
  LED_PPM_FAULT,
  LED_PRE_BIND,
  LED_BINDING,
  LED_RANGE_CHECK,
  LED_OFF
} LedMode_t;

typedef enum {
  NO_ERROR,              //0
  EEPROM_DIFF_ERROR,     //1 usw
  EEPROM_SUMM_ERROR,
  NO_PPM_ERROR,
  LAST_ERROR
} Error_t;

#define TICKS_PER_uS      1                 // number of timer ticks per 1 microsecond with prescaler = 8 and CPU 8MHz
#define MAX_CHANNELS      8                 // maximum number of channels we can store, don't increase this above 8
#define MIN_IN_PULSE  ( 750 * TICKS_PER_uS) // valid pulse must be at least   750us
#define MAX_IN_PULSE  (2250 * TICKS_PER_uS) // valid pulse must be less than 2250us
#define SYNC_GAP_LEN  (5000 * TICKS_PER_uS) // we assume a space at least 5000us is sync
#define VALID_FRAMES     10                 // must have this many consecutive valid frames to transition to the ready state.

#define DSM2_CHANNELS      6                // Max number of DSM2 Channels transmitted
#define BINDING_PIN        4                // Pin used to bind
#define BINDING_LED        5                // Pin used for Binding in process LED
#define PPM_OK_LED         6                // Pin used for PPM Ok signal LED
#define SIGNLE_LED         BINDING_LED      //
#define SLOW_FLASH_LED   100                // LED flash interval in ms
#define FAST_FLASH_LED    25                // LED flash interval in ms

#define ISR_DEBUG_PIN     11                // only to Debug the ISR


#define DSM_HEADER_DSMX         0x08
#define DSM_HEADER_EU           0x10
#define DSM_HEADER_BINDING      0x80
#define DSM_HEADER_RANGECHECK   0x20

#define BUTTON_PRESS            LOW
#define BUTTON_RELEASE          HIGH
#define BUTTON_UNDEFINED        3 

#define BUTTON_FILTER_RANGE     11

#define SECURITY_STRING         "Copyright Robert Proeschild look at www.demoboard.de"
#define SECURITY_SUMM           5013
static byte SECURITY_DIFF[] =  {0x43, 0x2C, 0x1F, 0x09, 0x0B, 0x1B, 0x0E, 0x0F, 0x1C, 0x54, 0x72, 0x3D, 0x0D, 0x07, 0x17,
                                0x06, 0x54, 0x70, 0x22, 0x1D, 0x0A, 0x16, 0x10, 0x0B, 0x01, 0x05, 0x08, 0x44, 0x4C, 0x03,
                                0x00, 0x04, 0x4B, 0x41, 0x15, 0x54, 0x57, 0x00, 0x00, 0x59, 0x4A, 0x01, 0x08, 0x02, 0x0D,
                                0x0D, 0x0E, 0x13, 0x16, 0x4A, 0x4A, 0x01, 0x65 };

//GRAUPNER_IFS_MC
#ifdef GRAUPNER_IFS_MC
  #define MODE           1
  #define OFFSET         873
  #define SCALE          0.95506
  static byte ChanIndex[] = {1,2,3,4,5,6};    //PPM to DSM2 Channel Mapping Table
  #define TCCR1B_VALUE   (1<<ICES1) | (1<<CS11)
#endif

//FUTABA
#ifdef FUTABA
  #define MODE           2
  #define OFFSET         1745
  #define SCALE         -0.81459
  static byte ChanIndex[] = {3,1,2,4,5,6};// PPM to DSM2 Channel Mapping Table
  #define TCCR1B_VALUE   (1<<CS11)
#endif

//MULTIPLEX
#ifdef MULTIPLEX
  #define MODE           3
  #define OFFSET         759
  #define SCALE          0.84367
  static byte ChanIndex[] = {4,1,2,3,5,6};    //PPM to DSM2 Channel Mapping Table                                              //Getestet mit Mode 2 Cockpit MM
  #define TCCR1B_VALUE   (1<<ICES1) | (1<<CS11)
#endif


static int Pulses[  MAX_CHANNELS + 1];      // array holding channel pulses width value in microseconds
static int Failsafe[MAX_CHANNELS + 1];      // array holding channel fail safe values
static byte ChannelNum;                     // number of channels detected so far in the frame (first channel is 1)
static byte ChannelCnt;                     // the total number of channels detected in a complete frame
static State_t State;                       // this will be one of the following states: Null, Not_Synched, Acquiring, Ready, Failsafe
static byte stateCount;                     // counts the number of times this state has been repeated

static byte DSM2_Header[2];
static byte DSM2_Channel[DSM2_CHANNELS*2] = {0x00,0xAA,0x05,0xFF,0x09,0xFF,0x0D,0xFF,0x13,0x54,0x14,0xAA};
static byte DSM2_SentFrame = 0;
  
static byte count;
static int pulse;

static unsigned long SlowTimerLED;
static unsigned long FastTimerLED;
static unsigned long TimerDsmSend;
static unsigned int ButtonFiler[BUTTON_FILTER_RANGE];
static unsigned int FilterIdx;
static unsigned int BadCount  = 0;
static unsigned int GoodCount = 0;


/* ---------- ---------- ---------- Sync ---------- ---------- ---------- */
void processSync() {                 // sync pulse was detected so reset the channel to first and update the system state
  Pulses[0] = ICR1 / TICKS_PER_uS;          // save the sync pulse duration for debugging
  if(State == READY)
  {                      
    if( ChannelNum != ChannelCnt)           // if the number of channels is unstable, go into failsafe
      State = FAILSAFE;                     
  }                                         
  else
  {                                    
    if(State == NOT_SYNCHED)
    {              
      State = ACQUIRING;                    // this is the first sync pulse, we need one more to fill the channel data array
      stateCount = 0;                       
    }
    else
    {                                
      if( State == ACQUIRING)
      {             
        if(++stateCount > VALID_FRAMES)
        {
          State = READY;                    // this is the second sync and all channel data is ok so flag that channel data is valid
          ChannelCnt = ChannelNum;          // save the number of channels detected
        }                                   
      }
      else if( State == FAILSAFE)
      {            
          if(ChannelNum == ChannelCnt)      // did we get good pulses on all channels?
            State = READY;                  
        }                                   
    }                                       
  }                                         
  ChannelNum = 0;                           // reset the channel counter
}

/* ---------- ---------- ---------- Interrupt ---------- ---------- ---------- */
ISR(TIMER1_OVF_vect) {

#ifdef DEBUG_PIN
  digitalWrite(ISR_DEBUG_PIN,  HIGH);   
  digitalWrite(ISR_DEBUG_PIN,  LOW);   
#endif

  if(State == READY) {
    State = FAILSAFE;                       // use fail safe values if signal lost
    ChannelNum = 0;                         // reset the channel count
  }                                         
}                                           
                                            
ISR(TIMER1_CAPT_vect) {                     // we want to measure the time to the end of the pulse
	
#ifdef DEBUG_PIN
  digitalWrite(ISR_DEBUG_PIN, HIGH);      
  digitalWrite(ISR_DEBUG_PIN, LOW);     
#endif

  TCNT1 = 0;                                // reset the counter
  if(ICR1 >= SYNC_GAP_LEN)                  // is the space between pulses big enough to be the SYNC
    processSync();                          
  else                                      
    if(ChannelNum < MAX_CHANNELS) {         // check if its a valid channel pulse and save it
      if((ICR1 >= MIN_IN_PULSE) && (ICR1 <= MAX_IN_PULSE)){  // check for valid channel data
        Pulses[++ChannelNum] = ICR1 / TICKS_PER_uS;         // store pulse length as microseconds
      }
      else
      {
        if(State == READY)
        {
          State = FAILSAFE;                 // use fail safe values if input data invalid
          ChannelNum = 0;                   // reset the channel count
          BadCount++;
        }
      }
    }

    if( ChannelNum == ChannelCnt) 
    {
      DSM2_SentFrame = 0;
      GoodCount++;
    }
}

/* ---------- ---------- ---------- Class ---------- ---------- ---------- */
class PPM_Decode {

public:
  PPM_Decode() {                            // Constructor
    // empty                                
  }                                         
                                            
  void begin() {                            
    pinMode(8, INPUT);                      // Timer1 interrupt handler uses pin 8 as input, do not change it
    ChannelNum = 0;                         
    State   = NOT_SYNCHED;                  
    TCCR1A  = 0x00;                         // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1
                                            //  PWM11=0,  PWM10=0 => PWM Operation disabled
    TCCR1B = TCCR1B_VALUE;        // capture using rising edge,  prescaler = 8
    TIMSK1 = _BV(ICIE1)|_BV (TOIE1);        // enable input capture and overflow interrupts for timer 1
    for(byte ch = 1; ch <= MAX_CHANNELS; ch++) {
      Failsafe[ch] = Pulses[ch] = 1500;     // set midpoint as default values for pulses and failsafe
    }
    Failsafe[3] = Pulses[3] = 1100;       // set channel 3 failsafe pulse width to min throttle
  }

  State_t getState() {
    return State;
  }

  byte getChannelCnt() {
    return ChannelCnt;
  }

  void  setFailsafe(byte ch, int value) {   // pulse width to use if invalid data, value of 0 uses last valid data
    if((ch > 0) && (ch <= MAX_CHANNELS))
      Failsafe[ch] = value;
  }

  void  setFailsafe() {                     // setFailsafe with no arguments sets failsafe for all channels to their current values
    if(State == READY)                      // useful to capture current tx settings as failsafe values
      for(byte ch = 1; ch <= MAX_CHANNELS; ch++)
        Failsafe[ch] = Pulses[ch];
  }

  int getChannelData(uint8_t channel) {     // this is the access function for channel data
    int result = 0;                         // default value
    if(channel <= MAX_CHANNELS)  {
      if((State == FAILSAFE) && (Failsafe[channel] > 0 ))
        result = Failsafe[channel];         // return the channels failsafe value if set and State is Failsafe
      else
        if((State == READY) || (State == FAILSAFE)) {
          cli();                            // disable interrupts
          result = Pulses[channel];         // return the last valid pulse width for this channel
          sei();                            // enable interrupts
        }
    }
    return result;
  }
};

PPM_Decode Receiver = PPM_Decode();

int CheckEeprom()
{
  byte         OldEeprom   = 0;
  byte         ValueEeprom = 0;
  unsigned int SumEeprom   = 0;
  byte         DiffEeprom  = 0;

  for (int adr = 0; adr < sizeof(SECURITY_STRING); adr++)
  {
    ValueEeprom = EEPROM.read(adr);
    SumEeprom = SumEeprom + ValueEeprom;

    //Diff
    DiffEeprom = OldEeprom ^ ValueEeprom;
    OldEeprom = ValueEeprom;

    if(DiffEeprom != SECURITY_DIFF[adr]) return EEPROM_DIFF_ERROR;
  }

  if (SumEeprom != SECURITY_SUMM) return EEPROM_SUMM_ERROR;
  return NO_ERROR;
}


unsigned long StartTimerMs(unsigned long ms)
{
  return (millis() + ms);
}

bool TimePassed(unsigned long Timer)
{
  if (Timer <= millis())
    return true;
  else
    return false;
}

void ErrorLED(byte Error){
  while(true){
    for (int count = Error; count; count--){
      digitalWrite(BINDING_LED, HIGH);
      digitalWrite(PPM_OK_LED, HIGH);
      delay(200);
      digitalWrite(BINDING_LED, LOW);
      digitalWrite(PPM_OK_LED, LOW);
      delay(500);
    }
    delay(4000);
  }
}

void SetLED(byte led_mode) {
  static bool slow_flasching;
  static bool fast_flasching;

  if(TimePassed(SlowTimerLED))
  {
    slow_flasching = !slow_flasching;
    SlowTimerLED  = StartTimerMs( SLOW_FLASH_LED );
  }               

#ifdef SINGLE_LED  
  if(TimePassed(FastTimerLED))
  {
    fast_flasching = !fast_flasching;
    FastTimerLED  = StartTimerMs( FAST_FLASH_LED );
  }               
#endif
  
  switch( led_mode )
  {

#ifdef SINGLE_LED
    case LED_PRE_BIND :
      digitalWrite(SIGNLE_LED, fast_flasching);
    break;    
    
    case LED_BINDING :
      digitalWrite(SIGNLE_LED, slow_flasching);
    break;
  
    case LED_PPM_OK :
      digitalWrite(SIGNLE_LED, LOW );     
    break;

    case LED_PPM_FAULT :
      digitalWrite(SIGNLE_LED, HIGH);     
    break;

    case LED_OFF :
      digitalWrite(SIGNLE_LED, LOW);     
    break;

#else
    case LED_PRE_BIND :
      digitalWrite(BINDING_LED, slow_flasching);
      digitalWrite(PPM_OK_LED,  !slow_flasching);      
    break;    
    
    case LED_BINDING :
      digitalWrite(BINDING_LED, slow_flasching);
      digitalWrite(PPM_OK_LED,  LOW);      
    break;
  
    case LED_PPM_OK :
      digitalWrite(BINDING_LED, LOW );     
      digitalWrite(PPM_OK_LED,  slow_flasching);
    break;

    case LED_PPM_FAULT :
      digitalWrite(BINDING_LED, HIGH);     
      digitalWrite(PPM_OK_LED,  LOW);   
    break;
    case LED_OFF :
      digitalWrite(BINDING_LED, LOW);     
      digitalWrite(PPM_OK_LED,  LOW);   
    break;
#endif  
  }
};

void SendNormalFrame( byte Header)
{
  //Set normal header
#ifdef FULLRANGE
  DSM2_Header[0] = Header;
#endif
#ifdef SHORTRANGE
  DSM2_Header[0] = 0x59;
  DSM2_Header[1] = 0x00;
#endif

  if(Receiver.getState() == READY || Receiver.getState() == FAILSAFE) {
    if(ChannelNum == 0 || ChannelNum == ChannelCnt) {   // during sync pulse or in failsafe
      if(DSM2_SentFrame == 0)                  // if DSM2 frame is not sent yet
      {
        for (byte i=0; i<DSM2_CHANNELS; i++) // get receiver data
        {
//          pulse = Receiver.getChannelData(ChanIndex[i]) - 1000; //Offset der min. Stellung
          pulse = int(Receiver.getChannelData(ChanIndex[i]) * SCALE) - OFFSET;
       
          pulse = constrain(pulse, 0, 0x3FF);
          DSM2_Channel[i*2]   = (byte)(i<<2) | highByte(pulse);
          DSM2_Channel[i*2+1] = lowByte(pulse);
        }
        sendDSM2();                         // send frame
        DSM2_SentFrame = 1;                      // frame sent flag
      } else {
        if(Receiver.getState() == FAILSAFE) {
          delay(20);                        // in case of failsafe
          DSM2_SentFrame = 0;                    // reset flag after delay
        }
      }
    } 
  }
}

void SendBindingFrame( byte Header)
{
#ifdef FULLRANGE
  DSM2_Header[0] = Header;
#endif
#ifdef SHORTRANGE
  DSM2_Header[0] = 0x80;
#endif
  DSM2_Header[1] = 0;
  sendDSM2();
}

unsigned int GetFilerBindingButton()
{
  unsigned int Result = BUTTON_UNDEFINED;
  
  ButtonFiler[FilterIdx] = digitalRead(BINDING_PIN);
  if( ++FilterIdx == BUTTON_FILTER_RANGE) FilterIdx = 0;
  
  int FilterSumme = 0;
  for( int i = 0; i < BUTTON_FILTER_RANGE; i++)
  {
    FilterSumme += ButtonFiler[i]; 
  }
  
  if (FilterSumme == 0)                   Result = LOW;
  if (FilterSumme == BUTTON_FILTER_RANGE) Result = HIGH;

  return Result;
}


void setup() {
  byte idx;
  byte mode;
  byte maxpulse;
  delay(100);

  pinMode(BINDING_PIN, INPUT);
  pinMode(BINDING_LED, OUTPUT);
  pinMode(PPM_OK_LED,  OUTPUT);
  pinMode(ISR_DEBUG_PIN,  OUTPUT);
  digitalWrite(BINDING_LED, LOW);
  digitalWrite(PPM_OK_LED, LOW);

  //Tell Version via LED
  mode = MODE;
#ifdef SHORTRANGE
  mode += 10;
#endif  
  maxpulse = max(VERSION, mode);
  for (idx = 0; idx <= maxpulse; idx++)
  {
     if(VERSION > idx) digitalWrite(BINDING_LED, HIGH);
     if(mode    > idx) digitalWrite(PPM_OK_LED, HIGH);
     delay(1); 
     digitalWrite(BINDING_LED, LOW);
     digitalWrite(PPM_OK_LED, LOW);
     delay(1); 
  } 
  delay(10); 


  #ifdef WRITE_EEPROM
    //Schreibt den securety string into EEPROM
    char securety[] = SECURITY_STRING;
    for (idx = 0; idx < sizeof(securety); idx++)
    {
      EEPROM.write(idx, securety[idx]); 
    }
  
    while (true)
    {
      digitalWrite(BINDING_LED, HIGH);
      delay(50);
      digitalWrite(BINDING_LED, LOW);
      delay(50);
    }
  #endif
  
  //Initialisieren der RS232
  #ifdef DEBUG
  Serial.begin(115200);      // print values on the screen
  #else
    #ifdef FULLRANGE
      Serial.begin(125000);    // closest speed for DSM2 module, otherwise it won't work
    #endif
    #ifdef SHORTRANGE
      Serial.begin(138000);    // closest speed for DSM2 module, otherwise it won't work
    #endif  
  #endif

  int CheckEepromResult = CheckEeprom(); 
  if (CheckEepromResult != NO_ERROR)
  {
    ErrorLED(CheckEepromResult);
  }
   
 
  //Init Button Filter
  for (int i = BUTTON_FILTER_RANGE; i; i--)
  {
    ButtonFiler[i -1] = digitalRead(BINDING_PIN);
  }
  FilterIdx = 0;


  //Starte PPM Erkennung
  Receiver.begin();
  //IO festelegen

  delay(100);

  //Warten auf ein stabiles Signal
  count = 50;
  while(Receiver.getState() != READY) // wait 5 sec or until PPM data is stable and ready
  {
    delay(100);
    SetLED(LED_PPM_FAULT);
    count--;
    if (count == 0)
    {
      ErrorLED(NO_PPM_ERROR);
    }
  }

  //Start Timer
  SlowTimerLED = StartTimerMs( 0 );
  TimerDsmSend = StartTimerMs( 0 );

  //Binding oder nicht?
  while(GetFilerBindingButton() == BUTTON_UNDEFINED);
  if(GetFilerBindingButton() == BUTTON_RELEASE) // Kein Binding
  {
    //Erste Frames senden
    #ifdef FULLRANGE
      DSM2_Header[0] = DSM_HEADER_DSMX | DSM_HEADER_EU;
      DSM2_Header[1] = 0x00;
    #endif
    #ifdef SHORTRANGE
      //One Time a sync
      DSM2_Header[0] = 0x58;
      DSM2_Header[1] = 0x00;
    #endif
    sendDSM2();
    delay(20);
  }
  else //Binding
  {              
    //Warten bis der Taster losgelassen wird
    while ((GetFilerBindingButton() == BUTTON_PRESS) || (GetFilerBindingButton() == BUTTON_UNDEFINED)) 
    {
      SetLED(LED_PPM_FAULT);
    }

    //Taster wurde los gelassen und warten auf erneutes dr�cken    
    while ((GetFilerBindingButton() == BUTTON_RELEASE) || (GetFilerBindingButton() == BUTTON_UNDEFINED))
    {
      SetLED(LED_PRE_BIND);
    }

    //Binding aktiv
    while ((GetFilerBindingButton() == BUTTON_PRESS) || (GetFilerBindingButton() == BUTTON_UNDEFINED))
    {
      SendBindingFrame( DSM_HEADER_DSMX | DSM_HEADER_EU | DSM_HEADER_BINDING );
      SetLED(LED_BINDING);
      delay(20);
    }
    
    //Ende Binding
  }
  
}



 ///////////////////////////////////////////////////////////////////////////
void loop() {
	if(GetFilerBindingButton() == BUTTON_PRESS)
	{
		SendNormalFrame( DSM_HEADER_DSMX | DSM_HEADER_EU | DSM_HEADER_RANGECHECK);
		SetLED(LED_PPM_FAULT);
	}
	else
	{
		SendNormalFrame( DSM_HEADER_DSMX | DSM_HEADER_EU);      
		if(Receiver.getState() == READY) SetLED(LED_PPM_OK);
		else                             SetLED(LED_PPM_FAULT);
	}

}

#ifndef DEBUG
void sendDSM2() {
    Serial.write(DSM2_Header, 2);
#ifdef SHORTRANGE
    DSM2_Channel [8] = 0x13;
    DSM2_Channel [9] = 0x54;
    DSM2_Channel[10] = 0x14;
    DSM2_Channel[11] = 0xAA;
#endif    
    Serial.write(DSM2_Channel, DSM2_CHANNELS*2);
}

#else
void sendDSM2() {
//    Serial.print(DSM2_Header[0], HEX);
//    Serial.print("   ");
//    Serial.print(DSM2_Header[1], HEX);
//    Serial.print("   ");
    for(byte i=0; i < DSM2_CHANNELS; i++) { // print channels 1 to 6 in Hex and Dec
      serialPrintHex(DSM2_Channel[i*2]);
      Serial.print(" ");
      serialPrintHex(DSM2_Channel[i*2+1]);
      Serial.print(" (");
      Serial.print((DSM2_Channel[i*2]&0x03)<<8 | DSM2_Channel[i*2+1], DEC);
      Serial.print(")  -");
      Serial.print(Receiver.getChannelData(i + 1), DEC);
      Serial.print("-  ");
    }
    Serial.print("  ");
    Serial.print(Receiver.getChannelData(0), DEC);  // sync pulse length
    
//    Serial.print(" G ");
//    Serial.print(GoodCount, DEC);  // sync pulse length
//    Serial.print(" B ");
//    Serial.print(BadCount, DEC);  // sync pulse length
    
    Serial.println(" ");
    delay(200);
}

void serialPrintHex(byte b) {
    byte b1 = (b >> 4) & 0x0F;
    byte b2 = (b & 0x0F);
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    Serial.print(c1);
    Serial.print(c2);
}
#endif
