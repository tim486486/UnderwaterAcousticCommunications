#include <AltSoftSerial.h>
#include <SimpleModbusSlaveAltSoft.h>
#include <SerialPrinting.h>
#include <avr/wdt.h>

bool doPrint = false;
bool doInitialize = false;

#define ID 2  //Modbus Slave ID
#define DEF_CT 60   //Communications Timeout
#define DEF_NP 5   //Default Number of Pulses
#define DEF_TT 127  //Default Timer Top, 159 = ~12.5kHz
#define DEF_GA 100  //Default Gain, 100 = ~1 
#define DEF_DP 850  //Default Data Points
#define DEF_BR 9600  //Baud Rate for Modbus Communications
#define DEF_DT 500   //Default delay time in ms
#define DEF_NS 5     //Default number of synchs to perform at each transmission
#define DEF_SE 1     //Default binary sequence, for transmissions
#define DEF_NC 1 //Default Number of Cycles of the Transmission Sequence
#define CMD_LENGTH 20 //Maximum Length of an Operator Command
#define MAX_CYCLES 20 //Maximum number of pulse cycles per transmission
#define LOOKUP_POINTS 10  //Number of lookup points in the transmission wave generator
#define invertTx true   //this will invert the transmission, to work with the DE pin on the MAX485
#define V_SOUND 1450.0  //Sound velocity used to determine expected travel time

// used to toggle the receive/transmit pin on the driver
#define calPin 2
#define hydro_out 3
#define ampPin 4
#define gControlPin 5
#define txEnablePin 7
#define rxPin 8
#define txPin 9
#define reservedByMB 10 // Reserved by the Modbus code
#define led 13
//AI0 Used for hydro_in

AltSoftSerial altSerial(invertTx); //Rx: 8, Tx: 9

//Modbus Table
enum 
{ 
  IN_SYNCH,    
  IN_HEARTBEAT,
  IN_TRANSMITTER, 
  IN_CONTROL,
  IN_DISTANCE,
  IN_SEQUENCE,
  IN_CMD,
  IN_CMD_INDEX,
  IN_DATA_INDEX,
  OUT_STATUS,
  OUT_CMD_INDEX,
  OUT_DATA_INDEX,
  OUT_HEARTBEAT,
  OUT_TIMEL,
  OUT_TIMEH,
  OUT_D1,
  OUT_D2,
  OUT_D3,
  OUT_D4,
  OUT_D5,
  HOLDING_REGS_SIZE //leave this last entry
};

//modes
enum{
  RUN,
  NOT_MUTE
};

//Status Word
enum {
  WAITING,
  RECEIVING,    
  TRANSMITTING,
  COMPLETE,
  SYNCHWAIT,
  SENDTIME,
  DATASEND,
  NUM_STATES,
  OP_CMD_RDY, // ready to accept a new operator command
  OP_CMD_ERR  // last operator command not recognized
};


//EEPROM Memory Map
enum{
  EEP_CT,
  EEP_CT1,
  EEP_NP,
  EEP_TT,
  EEP_GA,
  EEP_DP,
  EEP_DP1,
  EEP_BR,
  EEP_BR1,
  EEP_DT,
  EEP_DT1,
  EEP_NS,
  EEP_NC
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array

bool isRunning = false;

unsigned int currentState = WAITING;
unsigned int nextState = WAITING;

unsigned int s = (1 << WAITING) | (1 << OP_CMD_RDY);

bool forceTransition = true;

// these variables used for timed events
unsigned long tLastBroadcast = 0;

unsigned long tOld = 0;
unsigned long tLastPulse = 0;
unsigned long tOut = 0;
unsigned long tLastSynch = 0;
unsigned long tOffset = 0;
unsigned long tComplete = 0;
unsigned long tContinue = 0;
unsigned long tRunning = 0;

bool watchdogEnabled = false;
bool oneSecPulse = false; //this bit will pulse for once cycle, once per second, used for heartbeat and printing

char incomingCommand[CMD_LENGTH];
String currentCommand = "NONE";

float sampleRate = 76923.0; //sample rate in Hz
bool isReceiver = false;

volatile bool stopListening = false;
volatile bool sequenceWait = false;
volatile byte pulseCount = 0;
volatile byte lookupIndex = 0;

byte data[DEF_DP];
unsigned int dataI = 0;

byte one[LOOKUP_POINTS];
byte zero[LOOKUP_POINTS];

byte samplePeriod = 0;

//adjustable parameters
unsigned long commTimeout = DEF_CT; //millis
byte numPulses = DEF_NP; //number of pulses to transmit
byte timerTop = DEF_TT; // 159 = 10 kHz
byte halfTimerTop = timerTop/2;
byte gain = DEF_GA;
unsigned int numDataPoints = DEF_DP;
unsigned int baudRate = DEF_BR;
unsigned long delayTime = DEF_DT*1E3L;
byte maxSynchs = DEF_NS;
unsigned int binSequence = DEF_SE;
byte numCycles = DEF_NC;
byte sequenceLength = 1;

byte numSynchs = 0;
bool mute = true;
bool dataBit = true;

/*
 * Setup Code
 */
void setup()
{  
  if (doPrint) Serial.begin(19200);
  if (doInitialize) initialize();
  loadParameters();
  
  cli();
  modbus_configure(&altSerial, baudRate, SERIAL_8N1, ID, txEnablePin, HOLDING_REGS_SIZE, holdingRegs);

  pinMode(led, OUTPUT);
  pinMode(hydro_out, OUTPUT);
  pinMode(gControlPin, OUTPUT);
  pinMode(ampPin, OUTPUT);
  pinMode(calPin, OUTPUT);
  
  TCCR0A |= _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(CS01); //Timer 0 set up to use PWM for gain control, wiring.c modified to match new prescalar
  OCR0B = gain; 
  digitalWrite(ampPin, 0);

  //sampling configuration
  ADCSRA = 0;
  ADMUX |= (1 << REFS0) //set reference voltage
         | (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  for (int i = 0; i < numDataPoints; i++){
    data[i] = 0;
  }
  
  holdingRegs[IN_CMD_INDEX] = 999;
  holdingRegs[OUT_DATA_INDEX] = 999;
  
  sei();
} 


/*
 * Main program loop
 */
void loop()
{
  processModbusData();

  if (((millis() - tLastBroadcast) > (commTimeout*1E3L)) && !watchdogEnabled)enableWatchdog();

  if (millis() - tLastPulse > 1E3L){
    oneSecPulse = true;
    tLastPulse = millis();
  }

  if (isRunning) stateMachine();
  
  prepareModbusOutputs();
  
  if (oneSecPulse){
    if (doPrint){
//      printLong("Comms Timeout", commTimeout);  //  EEP_CT,
//      printUInt("Num Pulses", numPulses);       //  EEP_NP,
//      printUInt("Timer Top", timerTop);         //  EEP_TT,
//      printUInt("Gain", gain);                  //  EEP_GA,
//      printUInt("Data Points", numDataPoints); //  EEP_DP,
//      printUInt("Baud Rate", baudRate);         // EEP_BR,
//      printLong("Delay Time", delayTime);       //  EEP_DT,
//      printUInt("Number of Synchs", maxSynchs); //  EEP_DT,
//      printUInt("Packet", packet);       //  EEP_PA,
//      printUInt("Packet Size", packetSize);       //  EEP_PS,
//      printUInt("Sequence",binSequence);
//      printUInt("Sequence Length", sequenceLength);
//      printUInt("Number of Cycles", numCycles);
    }
    oneSecPulse = false;
  }
}

/*
 * Process the data received from the slaves
 */
void processModbusData(){
  //save all the old stuff before reading
  int oldCmdIndex = holdingRegs[IN_CMD_INDEX];
  unsigned int prevHB = holdingRegs[IN_HEARTBEAT];

  modbus_update();

  unsigned int control = holdingRegs[IN_CONTROL];
  
  if (!isRunning && (control & (1 << RUN))) switchToRun();
  if (isRunning && !(control & (1 << RUN))) switchToStop();
  mute = !(control & (1 << NOT_MUTE));

  binSequence = holdingRegs[IN_SEQUENCE];
  sequenceLength = 1;
  for (int i = 1; i < 16; i++){
    if ((1 << i) & binSequence) sequenceLength = i+1;
  }
  

  if (oldCmdIndex != holdingRegs[IN_CMD_INDEX]) receiveNewCmdChar();
  if (!holdingRegs[IN_CMD] &&  !(s & (1 << OP_CMD_RDY))){
    currentCommand = incomingCommand;
    processCommand();
  }

  //Set the onboard LED to toggle when a broadcast is received from master
  if (prevHB != holdingRegs[IN_HEARTBEAT]){
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
  unsigned long rTimeL = (tOut << 16L) >> 16L;
  unsigned long rTimeH = tOut >> 16L;
  holdingRegs[OUT_TIMEL] = rTimeL;
  holdingRegs[OUT_TIMEH] = rTimeH;
}

/*
 * Interprets the latest command from the Master
 */
void processCommand(){
  String cmd = currentCommand;
  s &= ~(1 << OP_CMD_ERR);
  //Reset the Unit
  if (cmd.startsWith("re") && !watchdogEnabled){
      enableWatchdog();
  }

  else if (cmd.startsWith("ct")){
    unsigned long ct = (unsigned long)(cmd.substring(3,cmd.length())).toInt();
    if (ct > 60L){
      commTimeout = ct;
      EEPROM.put(EEP_CT, commTimeout);
    }
  }


  else if (cmd.startsWith("np")){
    unsigned int np = (cmd.substring(3,cmd.length())).toInt();
    if (np >= 0){
      numPulses = np;
      EEPROM.write(EEP_NP, numPulses);
    }
  }

  else if (cmd.startsWith("tt")){
    unsigned int tt = (cmd.substring(3,cmd.length())).toInt();
    if (tt >= 0 && tt <= 255){
      timerTop = tt;
      halfTimerTop = timerTop/2;       
      buildWave();
      EEPROM.write(EEP_TT, tt);
    }
  }

  else if (cmd.startsWith("ga")){
    unsigned int ga = (cmd.substring(3,cmd.length())).toInt();
    if (ga >= 0 && ga <= 255){
      gain = ga;
      OCR0B = gain;
      EEPROM.write(EEP_GA, gain);
    }
  }

  else if (cmd.startsWith("dp")){
    unsigned int dp = (cmd.substring(3,cmd.length())).toInt();
    if (dp >= 0 && dp <= DEF_DP){
      numDataPoints = dp;
      EEPROM.put(EEP_DP, numDataPoints);
    }
  }

  else if (cmd.startsWith("br")){
      unsigned int br = (cmd.substring(3,cmd.length())).toInt();{
          if (br == 1200 || br == 2400 || br == 4800 || br == 9600){
            EEPROM.put(EEP_BR, br);
            if (!watchdogEnabled) enableWatchdog();
          }
    }
  }

  else if (cmd.startsWith("dt")){
      int dt = (cmd.substring(3,cmd.length())).toInt();{
      delayTime = dt*1E3L;
      EEPROM.put(EEP_DT, dt);
    }
  }

  else if (cmd.startsWith("ns")){
      unsigned int ns = (cmd.substring(3,cmd.length())).toInt();{
      maxSynchs = ns;
      EEPROM.write(EEP_NS, maxSynchs);
    }
  }

  else if (cmd.startsWith("nc")){
      unsigned int nc = (cmd.substring(3,cmd.length())).toInt();{
      numCycles = nc;
      EEPROM.write(EEP_NC, numCycles);
    }
  }

  else if (cmd.startsWith("in")){
    initialize();
    if (!watchdogEnabled) enableWatchdog();
  }
  
  else{
    s |= (1 << OP_CMD_ERR);
  }
  currentCommand = "NONE";
  s |= (1 << OP_CMD_RDY);
}
/*
 * 
 */
void initialize(){    
    EEPROM.put(EEP_CT, DEF_CT);
    EEPROM.write(EEP_NP, DEF_NP);
    EEPROM.write(EEP_TT, DEF_TT);
    EEPROM.write(EEP_GA, DEF_GA);
    EEPROM.put(EEP_DP, DEF_DP);
    EEPROM.put(EEP_BR, DEF_BR);
    EEPROM.put(EEP_DT, DEF_DT);
    EEPROM.write(EEP_NS,DEF_NS);
    EEPROM.write(EEP_NC,DEF_NC);
}

/*
 * 
 */
void loadParameters(){
  commTimeout = readEEPROM(EEP_CT);
  numPulses = EEPROM.read(EEP_NP);
  timerTop = EEPROM.read(EEP_TT);
  halfTimerTop = timerTop/2;
  buildWave();
  gain = EEPROM.read(EEP_GA);
  OCR0B = gain;
  numDataPoints = readEEPROM(EEP_DP);
  baudRate = readEEPROM(EEP_BR);  
  delayTime = 1E3L*readEEPROM(EEP_DT);
  maxSynchs = EEPROM.read(EEP_NS);
  numCycles = EEPROM.read(EEP_NC);
}

/*
 * 
 */
void switchToRun(){
    s &= (1 << OP_CMD_RDY) | (1 << OP_CMD_ERR);
    currentState = WAITING;
    nextState = WAITING;
    forceTransition = true;
    isRunning = true;    
}
/*
 * 
 */
void switchToStop(){
    if (currentState = RECEIVING || currentState == TRANSMITTING){
      TCCR2A = 0; //stop talking
      TCCR2B = 0; //stop talking
      ADCSRA = 0; //stop listening
      digitalWrite(ampPin, 0);
      enableModbus(); 
    }
    nextState = WAITING;  
    stateChange();
    isRunning = false; 
}

/*
 * 
 */
void clearData(){    
    for (int i = 0; i < numDataPoints; i++) data[i] = 0;
    dataI = 0;  
}

/*
 * Analog Sampling Setup
 */
void hear(){
  byte adcOffset;
  byte ADCSRA_R = (1 << ADATE)   //enable auto trigger
         | (1 << ADIE)     //enable interrupts when measurement complete
         | (1 << ADEN)     //enable ADC
         | (1 << ADSC)     //start ADC measurements
         | (1 << ADPS2);   
  cli();
  if (isReceiver){
    ADCSRA = ADCSRA_R;       //prescalar: 16, sample rate: 76,924 Hz 
    samplePeriod = 13;
    adcOffset = 12; //the first conversion takes 25 clock cycles, so add the offset here
  }
  else{
    ADCSRA = ADCSRA_R | (1 << ADPS0);   //prescalar: 32, sample rate: 38,462 Hz
    samplePeriod = 26;
    adcOffset = 24; //the first conversion takes 25 clock cycles, so add the offset here
  }  
  tRunning = micros() - tOffset + adcOffset;
  sei();
}

/*
 * 
 */
ISR(ADC_vect) {//when new ADC value ready   
  data[dataI] = ADCH;//get value from A0;
  dataI++;
  if (dataI == numDataPoints) dataI = 0;  
  tRunning += samplePeriod;
  if (sequenceWait && (tRunning >= tContinue)) sequenceWait = false;  
  else if (stopListening){
    ADCSRA = 0;
    tComplete = tRunning;
    stopListening = false;
  }
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
    s &= ~(1 << OP_CMD_RDY);
  }
  incomingCommand[index] = holdingRegs[IN_CMD];
}

/*
 * State machine to handle control of the transducer
 */
void stateMachine(){
  unsigned long tStamp,tElapsed = 0;
  float travelTime;
  unsigned int stopPulse = 2*numPulses;

  byte OCR2B_ = 0;
  byte TCCR2A_ = 0;
  byte TCCR2B_ = 0;
  byte TIMSK2_ = 0;
  
  if (!mute){
    OCR2B_ = halfTimerTop;
    TCCR2A_ = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B_ = (1 << WGM22) | (0 << CS22) | (0 << CS21) | (1 << CS20);
    TIMSK2_ = (1 << OCIE2A); 
  }

  switch (currentState){

      case WAITING:
        if (holdingRegs[IN_SYNCH] != 0){            
          if (holdingRegs[IN_TRANSMITTER] == ID) nextState = TRANSMITTING;
          else nextState = RECEIVING;
        }
      break;

      case RECEIVING:
        travelTime = 10000.0*holdingRegs[IN_DISTANCE]/V_SOUND;        
        tContinue = delayTime + min(1E6L, travelTime + (1E6 / sampleRate) * (numDataPoints - 100));
        sequenceWait = true;
        while (sequenceWait){};  
        nextState = COMPLETE;        
      break;
      
      case TRANSMITTING:
        OCR2B = OCR2B_; //precharge
        TCCR2A = TCCR2A_;
        TCCR2B = TCCR2B_;
        lookupIndex = 0;
        tContinue = delayTime;
        sequenceWait = true;
        while (sequenceWait){}; 
        if (!mute){
          for (int i = 0; i < min(numCycles,MAX_CYCLES); i++){                 
            for (int j = min(sequenceLength-1,16); j >=0; j--){
                dataBit = binSequence & (1 << j);
                pulseCount = 0; 
                TIMSK2 = TIMSK2_;           
                while (pulseCount < numPulses){}            
             }
          }
        }   
        TIMSK2 = 0;
        OCR2B = 0;
        TCCR2A = 0;
        TCCR2B = 0;
        lookupIndex = 0;
        pulseCount = 0;        
        tContinue = min(delayTime + 100E3L, micros() - tOffset + 1E3L);
        sequenceWait = true;
        while (sequenceWait){};          
        nextState = COMPLETE;     
      break;

      case COMPLETE:        
        if (holdingRegs[IN_SYNCH] == 0) nextState = SYNCHWAIT;
      break;

      case SYNCHWAIT:
        if (holdingRegs[IN_SYNCH] != 0) nextState = SENDTIME;
      break;

      case SENDTIME:
        if (holdingRegs[IN_SYNCH] == 0){
          if (numSynchs == maxSynchs) nextState = DATASEND;
          else nextState = SYNCHWAIT;
        }
      break;

      case DATASEND:
        if ((holdingRegs[IN_DATA_INDEX] < numDataPoints/10) && (holdingRegs[IN_DATA_INDEX] != holdingRegs[OUT_DATA_INDEX])){
          for (int i = 0; i < 5; i++){
            int ind = (dataI + 10*holdingRegs[IN_DATA_INDEX] + 2*i)%numDataPoints;
            unsigned int h = data[ind] << 8;
            holdingRegs[OUT_D1 + i] = h | data[(ind+1)%numDataPoints];
          }
          holdingRegs[OUT_DATA_INDEX] = holdingRegs[IN_DATA_INDEX];
        }
        if (holdingRegs[IN_DATA_INDEX] == numDataPoints/10) nextState = WAITING;
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
  s = ((s & 0xFF80) | 1 << nextState);
  
  switch (nextState){

        case WAITING:
          digitalWrite(ampPin, 0);
          holdingRegs[OUT_DATA_INDEX] = 999;
          clearData();
          isReceiver = false;
          altSerial.resetSynched(); 
          numSynchs = 0;
        break;
  
        case RECEIVING:
          tOffset = altSerial.getTOff();
          digitalWrite(ampPin, 1);
          isReceiver = true;                   
          disableModbus();         
          hear();                   
        break;
  
        case TRANSMITTING:
          tOffset = altSerial.getTOff();      
          disableModbus(); 
          hear();      
        break;

        case COMPLETE:
          stopListening = true;
          while (stopListening) {}
          digitalWrite(ampPin, 0);
          enableModbus();          
        break;

        case SYNCHWAIT:
          tLastSynch = altSerial.getTOff();
          altSerial.resetSynched();
        break;

        case SENDTIME:
          numSynchs++;
          tOut = altSerial.getTOff() - tOffset;
        break;

        case DATASEND:  
          tOut = tComplete;
        break;
  
        default:
        break;
      }
      currentState = nextState;
      forceTransition = false;
}

/*
 * Carrier interrupt based on Timer 2
 */
ISR(TIMER2_COMPA_vect){
    if (pulseCount == numPulses){
      TIMSK2 = 0;
      OCR2B = halfTimerTop; //precharge
    }
    else{
      if (dataBit) OCR2B = one[lookupIndex]; 
      else OCR2B = zero[lookupIndex]; 
      lookupIndex++;
      if (lookupIndex == LOOKUP_POINTS){
        lookupIndex = 0;
        pulseCount++;
      }
    }
}

/*
 * 
 */
 void buildWave(){
    //Waveform Generation
    OCR2A = timerTop;// = (16*10^6) / (8000*8) - 1 (must be <256)
    for (int i = 0; i < LOOKUP_POINTS; i++){
      float o = (sin(i*2.0*PI/LOOKUP_POINTS)) + 1;
      float z = (sin(-i*2.0*PI/LOOKUP_POINTS)) + 1;
      one[i] = (timerTop/2)*o;
      zero[i] = (timerTop/2)*z;
    }
 }



