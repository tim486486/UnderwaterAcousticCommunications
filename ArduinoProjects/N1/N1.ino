#include <SimpleModbusSlaveAltSoft.h>
#include <avr/wdt.h>
#include <AltSoftSerial.h>

#define invertTx true //this will invert the transmission, to work with the DE pin on the MAX485
AltSoftSerial altSerial(invertTx); //Rx: 8, Tx: 9

//Slave ID
#define ID 1

// used to toggle the receive/transmit pin on the driver
#define txEnablePin 7
#define rxPin 8
#define txPin 9
#define reservedByMB 10 // Reserved by the Modbus code
#define hydro_out 11
//#define reservedByFDet 12 // I may remove this
#define led 13
//AI0 Used for hydro_in

//Baud Rate
#define baud 38400

//communications timeout in seconds
#define TIMEOUT 60000

//Length of a tick
#define TICK 25L

#define MAX_LONG 4294967295L;

//macros for setting and clearing the bits in the status and control bytes
#define clear_bit(a,z) (a &= ~_BV(z))
#define set_bit(a,z) (a |= _BV(z))

//maximum number of characters in a single incoming command
#define CMD_LENGTH 10

//the number of instructions that can be stored from the master
#define NUM_INSTRUCTIONS 50

//number of samples to keep for the frequency calculator
#define SAMPLES 10

enum 
{     
  IN_HEARTBEAT,
  IN_TIMEL,
  IN_TIMEH,
  IN_FREQ,   
  IN_CMD,
  IN_CMD_INDEX,
  IN_CONTROL, 
  OUT_STATUS,
  OUT_CMD_INDEX,
  OUT_HEARTBEAT,
  OUT_FREQ,
  OUT_RTIMEL,
  OUT_RTIMEH,  
  HOLDING_REGS_SIZE //leave this last entry
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array

//Status Word
enum {
  WAITING,
  RECEIVING,      
  TRANSMITTING,
  NUM_STATES,
  RX,
  TX,
  IDL,
  OP_CMD_RDY, // ready to accept a new operator command
  OP_CMD_ERR  // last operator command not recognized
};

//Control Word
enum{
  CMD_T,
  CMD_R,
  CMD_W,
  AUTO,
  MANUAL,
  DELAY_TX
};

bool manual = true;

unsigned long entryTime[NUM_STATES];

unsigned int currentState = WAITING;
unsigned int nextState = WAITING;

unsigned int c = (1 << DELAY_TX); //control byte
unsigned int s = (1 << WAITING) | (1 << OP_CMD_RDY);

float fOut = 200.0; //array of frequencies for sending to nodes
unsigned long out_tSched = 0;
unsigned int out_status = 0;

unsigned int fIn = 0; //array of frequencies for receiving from nodes
unsigned long in_tSched = 0;
unsigned int in_status = 0;

bool forceTransition = true;

// these variables used for timed events
unsigned long tOld = 0;
unsigned long tMaster = 0;
unsigned long tOffset = 0;
unsigned long tRX = 0;
unsigned long tLastBroadcast = 0;

bool watchdogEnabled = false;
bool oneSecPulse = false; //this bit will pulse for once cycle, once per second, used for heartbeat and printing

//data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;//keeps time and sends vales to store in timer[] occasionally
int timer[SAMPLES];//sstorage for timing of events
int slope[SAMPLES];//storage for slope of events
unsigned int totalTimer;//used to calculate period
unsigned int period;//storage for period of wave
byte index = 0;//current storage index
float frequency;//storage for frequency calculations
int maxSlope = 0;//used to calculate max slope as trigger point
int newSlope;//storage for incoming slope data

//variables for decided whether you have a match
byte noMatch = 0;//counts how many non-matches you've received to reset variables if it's been too long

char incomingCommand[CMD_LENGTH];
String currentCommand = "NONE";
String dynamicInstructions[NUM_INSTRUCTIONS];
int dynamicInstructionsIndex = 0;

//adjustable parameters
float deadband = 0.1;
unsigned long tDelay = 5000L;
int timerTol = 10;//timer tolerance- adjust this if you need
byte slopeTol = 50;//slope tolerance- adjust this if you need
unsigned long commTimeout = 60000L;

bool debug = false;
bool mute = false;

/*
 * Setup Code
 */
void setup()
{	
  modbus_configure(&altSerial, baud, SERIAL_8N1, ID, txEnablePin, HOLDING_REGS_SIZE, holdingRegs);

  // modbus_update_comms(baud, byteFormat, id) is not needed but allows for easy update of the
  // port variables and slave id dynamically in any function.
  modbus_update_comms(baud, SERIAL_8N1, ID);
  
  pinMode(led, OUTPUT);
  pinMode(hydro_out, OUTPUT);

  holdingRegs[IN_CMD_INDEX] = 999;

  if (debug) Serial.begin(19200);
} 
/*
 * Main program loop
 */
void loop()
{
  processModbusData();

  if (((millis() - tLastBroadcast) > commTimeout) && !watchdogEnabled) enableWatchdog();

  if (!manual) autoMode();
  else manualMode();

  //update the one sec pulse bit
  if ((getMicros()/1000L) - tOld >= 1000){
    tOld = getMicros()/1000L;
    oneSecPulse = true;
  }
  
  prepareModbusOutputs();
  
  if (oneSecPulse){
    if (debug){ 
//    printHoldingRegs(); //for testing purposes, comment out when not being used to save resources
    Serial.println("--- Command Status ---");
    printString("Last", currentCommand);
    printString("Incoming", incomingCommand);

    Serial.println("--- Node State Machine ---"); 
    if (currentState == WAITING) Serial.println("S: WAITING");  
    if (currentState == RECEIVING) Serial.println("S: RECEIVING");
    if (currentState == TRANSMITTING) Serial.println("S: TRANSMITTING");
    
    Serial.println("--- Local Node Words ---");
    printUInt("Status", s);
    printUInt("Control",c);
    
    Serial.println("--- Transmission Scheduling ---");   
    printInt("fSched", fIn);   
    printInt("Fin", frequency);
    printLong("tSched", in_tSched);  
    printLong("t", getMicros());
     
    Serial.println("--- Heartbeats ---");
    printUInt("Master", holdingRegs[IN_HEARTBEAT]);

    Serial.println("--- Parameters ---");
    printFloat("Deadband", deadband);
    printLong("Transmit Delay", tDelay);
    printInt("Timer Tolerance", timerTol);
    printInt("Slope Tolerance", slopeTol);
    printLong("Comms Timeout", commTimeout); 
    }    
    oneSecPulse = false;
  }
}

/*
 * Process the data received from the slaves
 */
void processModbusData(){
  //save all the old stuff before reading
  int oldIndex = holdingRegs[IN_CMD_INDEX];
  unsigned int prevHB = holdingRegs[IN_HEARTBEAT];
  unsigned long tMasterOld = tMaster;
  
  modbus_update();
  
  //receive the latest time from the master
  unsigned long timeH = (unsigned long)holdingRegs[IN_TIMEH] << 16L;
  tMaster = timeH | holdingRegs[IN_TIMEL];
  
  if (tMasterOld != tMaster){
    tOffset = micros();
    in_tSched = tMaster + (tDelay*1000L);
  }
    
  fIn = holdingRegs[IN_FREQ];

  if (manual && (c & (1 << AUTO))) switchToAuto();
  if (!manual && (c & (1 << MANUAL))) switchToManual();

  if (oldIndex != holdingRegs[IN_CMD_INDEX]) receiveNewCmdChar();
  if (!holdingRegs[IN_CMD] &&  !(s & (1 << OP_CMD_RDY))){
    currentCommand = incomingCommand;
    processCommand();
  }

  //Set the onboard LED to toggle when a broadcast is received from master
  if (prevHB != holdingRegs[IN_HEARTBEAT]){
    PORTB ^= 32;
    c = holdingRegs[IN_CONTROL]; //only update the control word when a full set of data has been received
    tLastBroadcast = millis();
  }
}
/*
 * Update the Holding Registers that will be sent to the slaves
 */
void prepareModbusOutputs(){
   //toggle the MASTERTOGGLE register once per second
  holdingRegs[OUT_HEARTBEAT]++;
  holdingRegs[OUT_CMD_INDEX] = holdingRegs[IN_CMD_INDEX];
  holdingRegs[OUT_STATUS] = s;
  
  if (currentState == RECEIVING) holdingRegs[OUT_FREQ] = frequency;
  else if (currentState == TRANSMITTING) holdingRegs[OUT_FREQ] = fOut;
  else holdingRegs[OUT_FREQ] = 0;

  unsigned long rTimeL = (tRX << 16L) >> 16L;
  unsigned long rTimeH = tRX >> 16L;
  holdingRegs[OUT_RTIMEL] = rTimeL;
  holdingRegs[OUT_RTIMEH] = rTimeH;
}

/*
 * Interprets the latest command from the Master
 */
void processCommand(){
  String cmd = currentCommand;
  clear_bit(s, OP_CMD_ERR);
  //Reset the Unit
  if (cmd.startsWith("re") && !watchdogEnabled){
      enableWatchdog();
  }
  
  else if (cmd.startsWith("db")){
      float db = (cmd.substring(3,cmd.length())).toFloat();
      if (db > 0.0 && db < 1.0) deadband = db;
  }
  
  else if (cmd.startsWith("td")){
    unsigned long td = (unsigned long)(cmd.substring(3,cmd.length())).toInt();
    if (td > 1L) tDelay = td*1000L;
  }
  else if (cmd.startsWith("tt")){
    int tt = (cmd.substring(3,cmd.length())).toInt();
    if (tt > 1) timerTol = tt;//timer tolerance- adjust this if you need  
  }
  
  else if (cmd.startsWith("st")){
    int st = (cmd.substring(3,cmd.length())).toInt();
    if (st >= 0 && st <= 255) slopeTol = st;//slope tolerance- adjust this if you need
  }

  else if (cmd.startsWith("ct")){
    unsigned long ct = (unsigned long)(cmd.substring(3,cmd.length())).toInt();
    if (ct > 60L) commTimeout = ct*1000L;
  }

  else if (cmd.startsWith("tx")){
    if (manual) c = (1 << CMD_T);
  }

  else if (cmd.startsWith("rx")){
    if (manual) c = (1 << CMD_R);   
  }

  else if (cmd.startsWith("wa")){
     if (manual) c = (1 << CMD_W); 
  }
  
  else{
    set_bit(s, OP_CMD_ERR);
  }
  currentCommand = "NONE";
  set_bit(s, OP_CMD_RDY);
}
/*
 * 
 */
void switchToAuto(){
  
    s &= (1 << OP_CMD_RDY);
    c = (1 << DELAY_TX); //control byte
    currentState = WAITING;
    nextState = WAITING;
    forceTransition = true;
    manual = false;    
}
/*
 * 
 */
void switchToManual(){
    s &= (1 << OP_CMD_RDY);
    c = (1 << CMD_W);
    currentState = WAITING;
    nextState = WAITING;
    enableModbus();
    forceTransition = true;
    manual = true;       
}

/*
 * State machine to handle control of the transducer
 */
void autoMode(){

  switch (currentState){
      long rTime;
      
      case WAITING:
        if ((c & (1 << CMD_T)) && (getMicros() > (in_tSched - 100000L))){
          disableModbus();
          while (getMicros() < in_tSched){}
          fOut = fIn;
          if (!mute) speak();
          enableModbus();          
          nextState = TRANSMITTING;
          break;
        }
        if (c & (1 << CMD_R)&& (getMicros() > (in_tSched - 500000L))){
          nextState = RECEIVING;
          break;
        }
      break;

      case RECEIVING:
        rTime = entryTime[RECEIVING];
        while(!(s & (1 << RX)) && (abs(fIn - frequency) > (fIn*deadband)) && (rTime - entryTime[RECEIVING] < 10000000L)){
          if (period > 0) frequency = 38462.0/float(period);
          rTime = getMicros();
          tRX = rTime;
        }
        enableModbus();
        set_bit(s,RX);
        
        if (!(c & (1 << CMD_R))) nextState = WAITING;
      break; 
  
      case TRANSMITTING:
        if (!(c & (1 << DELAY_TX))){
          set_bit(s,TX);
          nextState = WAITING;
        }
        if (fOut != fIn){
          fOut = fIn;
          speak();
        }
      break;
  
      default:
        break;
    }

    if (forceTransition || ((currentState != nextState))) stateChange();
    
}

/*
 * 
 */
void stateChange(){

  switch (nextState){

        case WAITING:
          dontSpeak();
          dontHear();
          entryTime[WAITING] = getMicros();
          clear_bit(s,RECEIVING);
          clear_bit(s,TRANSMITTING);
          clear_bit(s,IDL);
          set_bit(s,WAITING);
          period = 0;
          frequency = 0.0;
          fIn = 0.0; 
          fOut = 0.0;  
        break;        
  
        case RECEIVING:
          hear();
          entryTime[RECEIVING] = getMicros(); 
          clear_bit(s,WAITING);
          clear_bit(s,TRANSMITTING);
          clear_bit(s,IDL);
          clear_bit(s,RX);
          clear_bit(s,TX);
          set_bit(s, RECEIVING);
          if (!manual) disableModbus();
        break;
  
        case TRANSMITTING:
          if (manual){
            fOut = fIn;
            speak(); 
          }
          entryTime[TRANSMITTING] = getMicros();
          clear_bit(s,WAITING);
          clear_bit(s,RECEIVING);
          clear_bit(s,IDL);
          clear_bit(s,RX);
          clear_bit(s,TX);
          set_bit(s,TRANSMITTING);
        break;
  
        default:
          break;
      }
      currentState = nextState;
      forceTransition = false;
}

/*
 * 
 */
void manualMode(){
  if (currentState == RECEIVING){
    if (period > 0) frequency = 38462.0/float(period); 
  }
  if (currentState == TRANSMITTING){
    if (fOut != fIn){
      fOut = fIn;
      speak();
    }
  }
  if (c & (1 << CMD_T)) nextState = TRANSMITTING;
  if (c & (1 << CMD_R)) nextState = RECEIVING;
  if (c & (1 << CMD_W)) nextState = WAITING;
  if (forceTransition || (currentState != nextState)) stateChange();
}

/*
   Carrier setup function
*/
void speak(){
  //Frequency Range (15 Hz, 62.5 kHz)

  unsigned int OCR2ATOP = 255;
  unsigned int prescalar;

  TCCR2A = _BV(COM2A0) | _BV(WGM20); //PWM, Phase-correct with OCRA Top

  if (fOut < 60.0){ //prescalar = 1024
    prescalar = 1024;
    TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21) | _BV(CS20); //clk prescalar = 1024, 15.625 kHz
  }

  else if (fOut < 500.0){ //prescalar = 256
    prescalar = 256;
    TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21); //clk prescalar = 256, 62.5 kHz
  }

  else if (fOut < 2000.0){ //prescalar = 128
    prescalar = 128;
    TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS20); //clk prescalar = 128, 125 kHz
  }

  else{ //prescalar = 64
    prescalar = 64;
    TCCR2B = _BV(WGM22) | _BV(CS22); //clk prescalar = 64, 125 kHz
  }
  OCR2ATOP = (16.0e+6) / (prescalar * 4.0 * fOut);
  if (OCR2ATOP > 255) OCR2ATOP = 255;
  if (OCR2ATOP < 1) OCR2ATOP = 1;
  OCR2A = OCR2ATOP;
}

/*
 * 
 */
void dontSpeak(){
  TCCR2A = 0;
  TCCR2B = 0;
  OCR2A = 0;
}

/*
 * Output setup function
 */
void hear(){
  //set up continuous sampling of analog pin 0 at 38.5kHz

  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
}

/*
 * 
 */
void dontHear(){
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  resetAll();
}

/*
 * 
 */
ISR(ADC_vect) {//when new ADC value ready
  
  //PORTB &= B11101111;//set pin 12 low
  prevData = newData;//store previous value
  newData = ADCH;//get value from A0
  if (prevData < 127 && newData >= 127){//if increasing and crossing midpoint
    newSlope = newData - prevData;//calculate slope
    if (abs(newSlope-maxSlope)<slopeTol){//if slopes are ==
      //record new data and reset time
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0){//new max slope just reset
        //PORTB |= B00010000;//set pin 12 high
        noMatch = 0;
        index++;//increment index
      }
      else if (abs(timer[0]-timer[index])<timerTol && abs(slope[0]-newSlope)<slopeTol){//if timer duration and slopes match
        //sum timer values
        totalTimer = 0;
        for (byte i=0;i<index;i++){
          totalTimer+=timer[i];
        }
        period = totalTimer;//set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;//set index to 1
        //PORTB |= B00010000;//set pin 12 high
        noMatch = 0;
      }
      else{//crossing midpoint but not match
        index++;//increment index
        if (index > 9){
          reset();
        }
      }
    }
    else if (newSlope>maxSlope){//if new slope is much larger than max slope
      maxSlope = newSlope;
      time = 0;//reset clock
      noMatch = 0;
      index = 0;//reset index
    }
    else{//slope not steep enough
      noMatch++;//increment no match counter
      if (noMatch>9){
        reset();
      }
    }
  }
    
  time++;//increment timer at rate of 38.5kHz
}

/*
 * 
 */
void reset(){//clear out some variables
  index = 0;//reset index
  noMatch = 0;//reset match couner
  maxSlope = 0;//reset slope
}

/*
 * 
 */
void resetAll(){
  //data storage variables
  newData = 0;
  prevData = 0;
  time = 0;//keeps time and sends vales to store in timer[] occasionally
  
  for (int i = 0; i < SAMPLES; i++){
    timer[i] = 0;//sstorage for timing of events
    slope[i];//storage for slope of events
  }

  totalTimer = 0;//used to calculate period
  period = 0;//storage for period of wave
  index = 0;//current storage index
  frequency = 0.0;//storage for frequency calculations
  maxSlope = 0;//used to calculate max slope as trigger point
  newSlope = 0;//storage for incoming slope data

//variables for decided whether you have a match
  noMatch = 0;//counts how many non-matches you've received to reset variables if it's been too long
}

/*
 * Enable the watchdog timer to reset the unit
 */
void enableWatchdog(){
  cli();      // disable all interrupts
  wdt_reset();// reset the WDT timer
  /*
  WDTCSR configuration:
  WDIE = 1: Interrupt Enable
  WDE = 1 :Reset Enable
  WDP3 = 0 :For 1000ms Time-out
  WDP2 = 1 :For 1000ms Time-out
  WDP1 = 1 :For 1000ms Time-out
  WDP0 = 0 :For 1000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
  sei();
  watchdogEnabled = true;
}
/*
 * Watchdog interrupt, put final instructions before resetting here
 */
ISR(WDT_vect) // Watchdog timer interrupt.
{ 
}

/*
 * Receievs a new character from the Master
 */
void receiveNewCmdChar(){
  int index = holdingRegs[IN_CMD_INDEX];
  if (!index){
    for (int i = 0; i < CMD_LENGTH; i++) incomingCommand[i] = 0;
    clear_bit(s, OP_CMD_RDY);
  }
  incomingCommand[index] = holdingRegs[IN_CMD];
}

/*
 * 
 */
unsigned long getMicros(){
  return (tMaster - tOffset + micros());
}


/*
 * Prints all the holding registers to the serial port
 */
void printHoldingRegs(){
  for (int i = 0; i < HOLDING_REGS_SIZE; i++){
    char c[20];
    sprintf(c, "holdingRegs[%d]", i);
    printUInt(c, holdingRegs[i]);
  }
}
/*
 * Used for testing, prints a new line in this format: varName: var
 */
void printChar(String varName, char var){
    Serial.print(varName);
    Serial.print(": ");
    Serial.print(var);
    Serial.print("\n");
}

/*
 * Used for testing, prints a new line in this format: varName: var
 */
void printString(String varName, String var){
    Serial.print(varName);
    Serial.print(": ");
    for (int i = 0; var[i]; i++) Serial.print(var[i]);
    Serial.print("\n");
}
/*
 * Used for testing, prints a new line in this format: varName: var
 */
void printInt(String varName, int var){
    Serial.print(varName);
    Serial.print(": ");
    Serial.print(var);
    Serial.print("\n");
}
/*
 * Used for testing, prints a new line in this format: varName: var
 */
void printFloat(String varName, float var){
    Serial.print(varName);
    Serial.print(": ");
    Serial.print(var);
    Serial.print("\n");
}

/*
 * Used for testing, prints a new line in this format: varName: var
 */
void printUInt(String varName, unsigned int var){
    Serial.print(varName);
    Serial.print(": ");
    Serial.print(var);
    Serial.print("\n");
}

/*
   Used for testing, prints a new line in this format: varName: var
*/
void printLong(String varName, unsigned long var) {
  Serial.print(varName);
  Serial.print(": ");
  Serial.print(var);
  Serial.print("\n");
}
