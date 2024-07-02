#include <AltSoftSerial.h>
#include <SimpleModbusMasterAltSoft.h>
#include <SerialPrinting.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <SD.h>

bool doPrint = false;
bool doInitialize = false;

#define invertTx true //this will invert the transmission, to work with the DE pin on the MAX485
AltSoftSerial altSerial(invertTx);//Rx: 8, Tx: 9

#define DEF_DI 0  //Default distance between nodes (m)
#define DEF_DE 0 //Default depth of nodes (cm)
#define DEF_TT 127  //Default Timer Top, 127 = ~12.5kHz
#define DEF_DP 850  //Default Data Points
#define DEF_BR 9600  //Modbus baud rate
#define DEF_PR 50 //Default Polling Rate
#define DEF_FT 1  //Default First Transmitter
#define DEF_GL 0  //Default Gain Level
#define DEF_NS 5  //Default Number of Synchs
#define DEF_SE 1 //Default Transmission Sequence
#define DEF_YR 19 //Default Year
#define DEF_MO 1  //Default Month
#define DEF_DA 1  //Default Day
#define DEF_HR 0  //Default Hour
#define DEF_MN 0  //Default Minute
#define LOOKUP_POINTS 10  //Number of lookup points in the transmission wave generator
#define SUPER_TIMEOUT 600E3L  //Supervisor state change timeout, 10 Minutes
#define N 2 //Number of Nodes in the network
#define CMD_LENGTH 20 //Maximum Length of an Operator Command
#define BROADCAST 0 //Modbus standard broadcast code
#define timeout 2000 //Milliseconds to wait for response from each slave
#define retry_count 255 //Number of times to poll each slave before marking it as failed
#define START_DELAY 5E3L  //Time to delay starting of the transmissions when switching to run (ms)
#define V_SOUND 1450  //Velccity of sound to use to calculate expected travel time (m/s)

// used to toggle the receive/transmit pin on the driver
#define shieldSelectPin 4
#define startPin 6
#define txEnablePin 7
#define led_write 12
#define led_run 13
#define reservedByMB1 44
#define reservedByMB2 45
#define txPin 46
#define rxPin 48
#define reservedbBySD1 50
#define reservedbBySD2 51
#define reservedbBySD3 52
#define sdSelectPin 53

//Modbus Table
enum
{
  OUT_SYNCH,        //Broadcast data
  OUT_HEARTBEAT,    
  OUT_TRANSMITTER,
  OUT_CONTROL,
  OUT_DISTANCE,
  OUT_SEQUENCE,
  OUT_N1_CMD,       //leave this as first output to node 1
  OUT_N1_CMD_INDEX,
  OUT_N1_DATA_INDEX,    
  IN_N1_STATUS,     //leave this as first input from node 1
  IN_N1_CMD_INDEX,
  IN_N1_DATA_INDEX,
  IN_N1_HEARTBEAT,
  IN_N1_TIMEL,
  IN_N1_TIMEH,
  IN_N1_D1,
  IN_N1_D2,
  IN_N1_D3,
  IN_N1_D4,
  IN_N1_D5,
  OUT_N2_CMD,       //leave this as first output to node 2
  OUT_N2_CMD_INDEX,
  OUT_N2_DATA_INDEX,
  IN_N2_STATUS,     //leave this as first input from node 2
  IN_N2_CMD_INDEX,
  IN_N2_DATA_INDEX,
  IN_N2_HEARTBEAT,
  IN_N2_TIMEL,
  IN_N2_TIMEH,
  IN_N2_D1,
  IN_N2_D2,
  IN_N2_D3,
  IN_N2_D4,
  IN_N2_D5,
  HOLDING_REGS_SIZE //leave this last entry
};

// Create an array of Packets to be configured
Packet packets[(2*N)+1];

// Masters register array
unsigned int holdingRegs[HOLDING_REGS_SIZE];

//modes
enum{
  RUN,
  NOT_MUTE,
  SD_WRITE
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

//Supervisor States
enum{
  ASSIGN_ROLES,
  CLEAR_ROLES,
  INIT_TIMESYNCH,
  CAPTURE_TIMESYNCH,
  CAPTURE_DATA
};

struct Node{
  bool newMBData = false;
  unsigned int state;
  unsigned int data[5];
  unsigned int dIin = 0;
  unsigned int dIout = 0;
  unsigned long tIn;
  int opCmd[CMD_LENGTH];
  int opCmdIndex = -1;
};

struct Node nodes[N];

//EEPROM Memory Map
enum{
  EEP_DIST, //distance between nodes in meters
  EEP_DIST1,
  EEP_TT,
  EEP_DP,
  EEP_DP1,
  EEP_PR,
  EEP_PR1, 
  EEP_DE, //depth in cm
  EEP_DE1,
  EEP_BR,
  EEP_BR1,
  EEP_FT,
  EEP_GL,
  EEP_NS,
  EEP_SE,
  EEP_SE1,
  EEP_YR,
  EEP_MO,
  EEP_DA,
  EEP_HR,
  EEP_MN,
  EEP_SC
};

bool isRunning = false;
bool allowRun = false;

// these variables used for timed events
unsigned long tOld = 0;
unsigned long tLastStateChange = 0; //time of last state change in auto mode
bool twoSecPulse = false; //this bit will pulse for once cycle, once per second, used for heartbeat and printing

bool rScheduled = false;
bool watchdogEnabled = false;
unsigned long rTimer = 0L; //time that the watchdog was started
unsigned long rDelay = 10E3L; //time to wait before resetting 
unsigned long tStart = 0L;
unsigned long tSet = 0L;
unsigned long tRunning = 0L;

//tunable parameters
unsigned int numDataPoints = DEF_DP;
unsigned int distance = DEF_DI;
unsigned int timerTop = DEF_TT;
float freq = 16E6 / ((DEF_TT + 1) * 10);
unsigned int pollingRate = DEF_PR;
unsigned int depth = DEF_DE;
unsigned int baudRate = DEF_BR;
byte firstTransmitter = DEF_FT;
byte gainLevel = DEF_GL;
byte maxSynchs = DEF_NS;
unsigned int binSequence = DEF_SE;

unsigned int control = (1 << NOT_MUTE) | (1 << SD_WRITE);
unsigned int currentState;
unsigned int nextState;
unsigned int transmitter;
bool forceTransition = false;

bool runSwitch = true;
bool debounce = false;
unsigned long debTimer = 0;

String lastCommand = "None";

char date[9];
char tm[7];
byte year = DEF_YR;
byte month = DEF_MO;
byte day = DEF_DA;
byte hour = DEF_HR;
byte minute = DEF_MN;
byte second = 0;

String currentFile;

bool pcConnected = false;

/*
 * Setup code
 */

void setup()
{
  if (doInitialize) initialize();
  loadParameters();
  
  // Broadcast Outputs
  modbus_construct(&packets[0], BROADCAST, PRESET_MULTIPLE_REGISTERS, OUT_SYNCH, OUT_N1_CMD - OUT_SYNCH, OUT_SYNCH);
  //Node 1 Outputs
  modbus_construct(&packets[1], 1, PRESET_MULTIPLE_REGISTERS, OUT_N1_CMD, IN_N1_STATUS - OUT_N1_CMD, OUT_N1_CMD);
  //Node 1 Inputs
  modbus_construct(&packets[2], 1, READ_HOLDING_REGISTERS, IN_N1_STATUS, OUT_N2_CMD - IN_N1_STATUS, IN_N1_STATUS);
  //Node 2 Outputs
  modbus_construct(&packets[3], 2, PRESET_MULTIPLE_REGISTERS, OUT_N1_CMD, IN_N2_STATUS - OUT_N2_CMD, OUT_N2_CMD);
  //Node 2 Inputs
  modbus_construct(&packets[4], 2, READ_HOLDING_REGISTERS, IN_N1_STATUS, HOLDING_REGS_SIZE - IN_N2_STATUS, IN_N2_STATUS);

  // Initialize the Modbus Finite State Machine
  modbus_configure(&altSerial, baudRate, SERIAL_8N1, timeout, pollingRate, retry_count, txEnablePin, packets, (2*N)+1, holdingRegs);

  pinMode(led_write, OUTPUT);
  pinMode(led_run, OUTPUT);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(sdSelectPin, OUTPUT);
  
  digitalWrite(led_write,LOW);
  digitalWrite(led_run,LOW);
  digitalWrite(sdSelectPin,HIGH);

  TCCR0B = _BV(CS01); //Timer 0 set up to use PWM for gain control, wiring.c modified to match new prescalar
  
  holdingRegs[OUT_N1_CMD_INDEX] = 999;
  holdingRegs[OUT_N2_CMD_INDEX] = 999;

  Serial.begin(19200);

  pcConnected = Serial;

  if (!SD.begin(shieldSelectPin)) control &= ~(1 << SD_WRITE); //Disable SD Card
}

/*
 * Main program loop
 */

void loop()
{    
  if (millis() > 10E3) allowRun = true;

  unsigned int prevHB[2];
  prevHB[0] = holdingRegs[IN_N1_HEARTBEAT];
  prevHB[1] = holdingRegs[IN_N2_HEARTBEAT];

  modbus_update();

  if (prevHB[0] != holdingRegs[IN_N1_HEARTBEAT]) nodes[0].newMBData = true;
  if (prevHB[1] != holdingRegs[IN_N2_HEARTBEAT]) nodes[1].newMBData = true;

  //update the one sec pulse bit
  if (millis() - tOld >= 2E3) {
    tOld = millis();
    twoSecPulse = true;
  }
  
  if (!watchdogEnabled && rScheduled && ((millis() - rTimer) > rDelay)) enableWatchdog();
  
  processMBInputs();
  
  if (Serial.available()) incomingTextCommand();
  if (isRunning && ((millis() - tStart) > START_DELAY)) supervisor();  
  
  updateMBOutputs();

  for (int i = 0; i < N; i++) nodes[i].newMBData = false;

  bool startPin_ = digitalRead(startPin);

  if (!debounce && (runSwitch != startPin_)){
    runSwitch = startPin_;    
    debTimer = millis();
    debounce = true;
  }

  if (debounce && ((millis() - debTimer) > 1E3L)) debounce = false;

  if (!isRunning && allowRun && !runSwitch) switchToRun();
  if (isRunning && runSwitch) switchToStop();

  if (twoSecPulse){
    updateDT(2);
    if (pcConnected && (doPrint || !isRunning)){ 
      Serial.println("\n--- Master ---");
      printLong("Running Time(s)", millis()/1E3L);
      Serial.println("Date and Time: " + String(date) + "_" + String(tm));
      
      Serial.println("\n--- Node 1 ---");
      printUInt("Heartbeat", holdingRegs[IN_N1_HEARTBEAT]);
      if (nodes[0].state & (1 << WAITING)) Serial.println("State: WAITING");  
      if (nodes[0].state & (1 << RECEIVING))Serial.println("State: RECEIVING");
      if (nodes[0].state & (1 << TRANSMITTING)) Serial.println("State: TRANSMITTING");
      if (nodes[0].state & (1 << COMPLETE)) Serial.println("State: COMPLETE");
      if (nodes[0].state & (1 << DATASEND)) Serial.println("State: DATASEND");
      if (nodes[0].state & (1 << OP_CMD_RDY)) Serial.println("Node 1 Ready");
      else Serial.println("Node  1 Processing...");
      if (nodes[0].state & (1 << OP_CMD_ERR))Serial.println("Last Command Not Recognized");
      Serial.println("\n--- Node 2 ---");
      printUInt("Heartbeat", holdingRegs[IN_N2_HEARTBEAT]);
      if (nodes[1].state & (1 << WAITING)) Serial.println("State: WAITING");
      if (nodes[1].state & (1 << RECEIVING)) Serial.println("State: RECEIVING");
      if (nodes[1].state & (1 << TRANSMITTING)) Serial.println("State: TRANSMITTING");
      if (nodes[1].state & (1 << COMPLETE)) Serial.println("State: COMPLETE");
      if (nodes[1].state & (1 << DATASEND)) Serial.println("State: DATASEND");
      if (nodes[1].state & (1 << OP_CMD_RDY)) Serial.println("Node 2 Ready");
      else Serial.println("Node  2 Processing...");
      if (nodes[1].state & (1 << OP_CMD_ERR)) Serial.println("Last Command Not Recognized");
      Serial.println("\n--- Control Parameters ---");
      if (control & (1 << NOT_MUTE)) Serial.println("Transmissions Active");
      else Serial.println("Transmissions Muted");
      if (control & (1 << SD_WRITE)) Serial.println("SD Card Enabled");
      else Serial.println("SD Card Disabled");
      printUInt("Gain Level", gainLevel);
      printUInt("First Transmitter", firstTransmitter);
      printUInt("Depth(cm)", depth);
      printUInt("Distance(cm)", distance);
      printFloat("Expected Travel Time(ms)", 10.0*distance/V_SOUND);
      printUInt("Timer Top", timerTop);
      printFloat("Freq(Hz)", freq);
      printUInt("Data Points", numDataPoints);
      printUInt("Baud Rate", baudRate);
      printUInt("Polling Rate(ms)", pollingRate);
      printUInt("Number of Synchs", maxSynchs);
      printUInt("Sequence",binSequence);
      Serial.print("Last Command: ");
      Serial.print(lastCommand);
      Serial.print("\n");
    }    
    twoSecPulse = false;
  }  
}



/*
 * Process the data received from the slaves
 */
void processMBInputs(){
  if (nodes[0].newMBData){
    if (holdingRegs[IN_N1_CMD_INDEX] == nodes[0].opCmdIndex){
      nodes[0].opCmdIndex++;
      if (nodes[0].opCmd[nodes[0].opCmdIndex] == 0){
        nodes[0].opCmdIndex = -1;
        holdingRegs[OUT_N1_CMD] = 0;
      }
    }

    //data collection
    nodes[0].dIin = holdingRegs[IN_N1_DATA_INDEX];
    for (int i = 0; i < 5; i++) nodes[0].data[i] = holdingRegs[IN_N1_D1 + i];
    
    nodes[0].state = holdingRegs[IN_N1_STATUS];

    unsigned long tH = (unsigned long)holdingRegs[IN_N1_TIMEH] << 16L;
    nodes[0].tIn = tH | holdingRegs[IN_N1_TIMEL];
  }

  if (nodes[1].newMBData){
    if (holdingRegs[IN_N2_CMD_INDEX] == nodes[1].opCmdIndex){
      nodes[1].opCmdIndex++;
      if (nodes[1].opCmd[nodes[1].opCmdIndex] == 0){
        nodes[1].opCmdIndex = -1;
        holdingRegs[OUT_N2_CMD] = 0;
      }
    }

    //data collection
    nodes[1].dIin = holdingRegs[IN_N2_DATA_INDEX];
    for (int i = 0; i < 5; i++) nodes[1].data[i] = holdingRegs[IN_N2_D1 + i];
    
    nodes[1].state = holdingRegs[IN_N2_STATUS];

    unsigned long tH = (unsigned long)holdingRegs[IN_N2_TIMEH] << 16L;
    nodes[1].tIn = tH | holdingRegs[IN_N2_TIMEL];
  }
}

/*
 * Update the Holding Registers that will be sent to the slaves
 */
void updateMBOutputs(){

   //Broadcast Data
   holdingRegs[OUT_HEARTBEAT]++;
   holdingRegs[OUT_DISTANCE] = distance;
   holdingRegs[OUT_SEQUENCE] = binSequence;
   holdingRegs[OUT_CONTROL] = control;

   //Node 1

   if (nodes[0].opCmdIndex != -1){
      holdingRegs[OUT_N1_CMD] = nodes[0].opCmd[nodes[0].opCmdIndex];
      holdingRegs[OUT_N1_CMD_INDEX] = nodes[0].opCmdIndex;
   }

   holdingRegs[OUT_N1_DATA_INDEX] = nodes[0].dIout;

   //Node 2

   if (nodes[1].opCmdIndex != -1){
      holdingRegs[OUT_N2_CMD] = nodes[1].opCmd[nodes[1].opCmdIndex];
      holdingRegs[OUT_N2_CMD_INDEX] = nodes[1].opCmdIndex;
   }

   holdingRegs[OUT_N2_DATA_INDEX] = nodes[1].dIout;
}

/*
 * 
 */
void switchToRun(){
  isRunning = true;
  control |= 1 << RUN;
  tLastStateChange = millis();
  tStart = millis();

  //Set the initial transmitter
  transmitter = firstTransmitter;
  currentState = ASSIGN_ROLES;
  nextState = ASSIGN_ROLES;
  forceTransition = true;

  //Turn on the running lamp
  PORTB |= (1 << PB7); 
}

/*
 * 
 */
 void switchToStop(){
    wait(1000);
    if (Serial) Serial.println('*'); //code to close the file  
    wait(1000);
    isRunning = false;
    control &= ~(1 << RUN);
    holdingRegs[OUT_SYNCH] = 0;
    holdingRegs[OUT_TRANSMITTER] = 0;  

    //Turn off the running and SD Lamps
    PORTB &= ~((1 << PB6) | (1 << PB7));
 }

/*
 *  
 */ 

void supervisor(){
      String buf = "";
    
      switch (currentState)
      {
        case ASSIGN_ROLES: 
          if  (allNodes(COMPLETE)) nextState = CLEAR_ROLES;
        break;

        case CLEAR_ROLES:
          if (allNodes(SYNCHWAIT)) nextState = INIT_TIMESYNCH;
        break;

        case INIT_TIMESYNCH:
          if (allNodes(SENDTIME)) nextState = CAPTURE_TIMESYNCH;
        break;
          
        case CAPTURE_TIMESYNCH: 
          if (allNodes(DATASEND)) nextState = CAPTURE_DATA;
          if (allNodes(SYNCHWAIT)) nextState = INIT_TIMESYNCH;
        break;

        case CAPTURE_DATA: 
          for (int i = 0; i < N; i++){
            if (nodes[i].dIin == nodes[i].dIout){
              buf = String(i+1) + ',';
              for (int j = 0; j < 5; j++){
                byte h = nodes[i].data[j] >> 8;
                byte l = (nodes[i].data[j] << 8) >> 8;
                buf = buf + String(h) + ',' 
                + String(l) + ',';
              }
              printData(buf + "\n");
              buf = "";
              nodes[i].dIout++;
            }
          }
          if (allNodes(WAITING)){
            transmitter++;
            if (transmitter > N) transmitter = 1;
            nextState = ASSIGN_ROLES;
          }
        break;

        default:
        break;
      }
      
      if (forceTransition || (nextState != currentState)) stateChange();
}
/*
 * 
 */
 void stateChange(){       
      tLastStateChange = millis();
      String buf = "";
    
      switch (nextState)
      {                    
        case ASSIGN_ROLES: 
          holdingRegs[OUT_SYNCH] = 7962;
          holdingRegs[OUT_TRANSMITTER] = transmitter;
          for (int i = 0; i < N; i++) nodes[i].dIout = 0; 
          
          wait(1000);
          if (Serial) Serial.println(';'); //start the file
          currentFile = createFile();
          wait(1000);
          buf = String(numDataPoints) + ',' 
          + String((int)freq) + ',' 
          + String(distance) + ',' 
          + String(depth) + ',' 
          + String(maxSynchs) + ',' 
          + String(binSequence) 
          + ",\n";
          printData(buf);                 
        break;
  
        case CLEAR_ROLES:
          holdingRegs[OUT_SYNCH] = 0;
          holdingRegs[OUT_TRANSMITTER] = 0;                
        break;
  
        case INIT_TIMESYNCH:
          holdingRegs[OUT_SYNCH] = 7962;                
        break;
         
        case CAPTURE_TIMESYNCH: 
          holdingRegs[OUT_SYNCH] = 0;
          
          for (int i = 0; i < N; i++){
              buf = String(i+1) + ',' 
              + String(nodes[i].tIn) + ',';
              printData(buf);
              buf = "";
          }
          printData("\n");
        break;
  
        case CAPTURE_DATA:
          buf = String(transmitter) + ',' 
          + String(nodes[transmitter-1].tIn) + ',';              
          for (int i = 1; i <= N; i++){
            if (i != transmitter){
              buf = buf + String(i) + ',' 
              + String(nodes[i-1].tIn) + ',';
              printData(buf);
              buf = "";
            }
          }
          printData("\n");
        break;
  
        default:
        break;
      }                                   
      currentState = nextState;
      forceTransition = false;                                    
      if (!rScheduled && ((millis() - tLastStateChange) > SUPER_TIMEOUT)) reset(1E3);    
}

/*
 * 
 */
bool allNodes(int state){
  bool flag = true;
  for (int i = 0; i < N; i++) if (!(nodes[i].state & (1 << state))) flag = false;
  return flag; 
}

/*
 * helper
 */
void reset(unsigned long delayTime){
  rScheduled = true;
  rTimer = millis();
  rDelay = delayTime;
}

/*
 * Enable the watchdog timer to reset the unit
 */
void enableWatchdog(){
  if (Serial) Serial.println("Resetting...");
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
  WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
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
 * Receives a text command from the Serial Monitor and converts it to an array of opCmd codes to send to the slave
 */
void incomingTextCommand(){
  bool valid = true;
  
  String entry,cmd; 
  unsigned long toNode;
  
  while (Serial.available()) entry = Serial.readString();

  lastCommand = entry;

  cmd = entry.substring(2,4);
  toNode = entry.substring(0,1).toInt();

  if (toNode == 0 || toNode > N+1) valid = false;
  
  if (valid){
    if (cmd.equals("da")){
      unsigned int y = entry.substring(5,7).toInt();
      unsigned int m = entry.substring(7,9).toInt();
      unsigned int d = entry.substring(9,entry.length()).toInt();
      year = y;
      month = m;
      day = d;
      EEPROM.write(EEP_YR,year);
      EEPROM.write(EEP_MO,month);
      EEPROM.write(EEP_DA,day);      
    }
    else if (cmd.equals("mi")){
      unsigned int h = entry.substring(5,7).toInt();
      unsigned int m = entry.substring(7,entry.length()).toInt();
      hour = h;
      minute = m;
      second = 0;
      EEPROM.write(EEP_HR,hour);
      EEPROM.write(EEP_MN,minute); 
      tSet = millis();   
    }
    else if (cmd.equals("mu")){
      if (control & (1 << NOT_MUTE)) control &= ~(1 << NOT_MUTE);
      else control |= 1 << NOT_MUTE;
    }
    else if (cmd.equals("sd")){
      if (control & (1 << SD_WRITE)) control &= ~(1 << SD_WRITE);
      else if (SD.begin(shieldSelectPin)) control |= 1 << SD_WRITE;
    }
    else if (cmd.equals("se")){
      unsigned int se = (entry.substring(5,entry.length())).toInt();
      binSequence = se;
      EEPROM.put(EEP_SE, binSequence);        
    }
    else if (cmd.equals("di")){
      unsigned int di = (entry.substring(5,entry.length())).toInt();
      distance = di;
      EEPROM.put(EEP_DIST, distance);        
    }
    else if (cmd.equals("pr")){
      unsigned int pr = (entry.substring(5,entry.length())).toInt();
      EEPROM.put(EEP_PR, pr);
      reset(10E3);
    }
    else if (cmd.equals("de")){
      unsigned int de = (entry.substring(5,entry.length())).toInt();
      depth = de;
      EEPROM.put(EEP_DE, depth);
    }
    else if (cmd.equals("ft")){
      unsigned int ft =  (entry.substring(5,entry.length())).toInt();
      firstTransmitter = ft;
      EEPROM.write(EEP_FT,firstTransmitter);
    }
    else if (cmd.equals("hi")){
      for (int i = 0; i < N; i++) sendCommand(i, "3,br,9600");//Send Command to all nodes
      EEPROM.put(EEP_BR, 9600);
      EEPROM.put(EEP_PR, 50);
      reset(60E3);  
    }
    else if (cmd.equals("lo")){
      for (int i = 0; i < N; i++) sendCommand(i, "3,br,2400");//Send Command to all nodes
      EEPROM.put(EEP_BR, 2400);
      EEPROM.put(EEP_PR, 200);
      reset(60E3);  
    }
    else if (cmd.equals("gl")){
      unsigned int gl = (entry.substring(5,entry.length())).toInt();
      String buf[2] = {"",""};
      int g[2] = {100,100};
      gainLevel = gl;
      if (gainLevel ==  1) {g[0] = 120; g[1] = 120;}
      if (gainLevel ==  2) {g[0] = 155; g[1] = 130;}
      if (gainLevel ==  3) {g[0] = 165; g[1] = 137;}
      if (gainLevel ==  4) {g[0] = 170; g[1] = 140;}
      if (gainLevel ==  5) {g[0] = 175; g[1] = 145;}
      if (gainLevel ==  6) {g[0] = 180; g[1] = 150;}
      if (gainLevel ==  7) {g[0] = 185; g[1] = 153;}
      if (gainLevel ==  8) {g[0] = 190; g[1] = 156;}
      if (gainLevel ==  9) {g[0] = 195; g[1] = 159;}
      if (gainLevel == 10) {g[0] = 200; g[1] = 162;}
      for (int i = 0; i < N; i++){
        buf[i].concat(i+1);
        buf[i].concat(",ga,");
        buf[i].concat(g[i]);
        sendCommand(i,buf[i]);
      }
      EEPROM.write(EEP_GL,gainLevel);
    }  

    else{
      if (toNode == N+1) for (int i = 0; i < N; i++) sendCommand(i, entry);//Send Command to all nodes
      else sendCommand(toNode - 1,entry); //Send Command to a single node

      if (cmd.equals("re")) reset(10E3);
      if (cmd.equals("tt")){
        timerTop = (entry.substring(5,entry.length())).toInt();
        EEPROM.write(EEP_TT, timerTop);
        freq = 16E6/((timerTop+1)*LOOKUP_POINTS); 
      }
      if (cmd.equals("dp")){
        unsigned int dp = (entry.substring(5,entry.length())).toInt();
        numDataPoints = dp;
        EEPROM.put(EEP_DP, numDataPoints);
      }
      if (cmd.equals("br")){
        unsigned int br = (entry.substring(5,entry.length())).toInt();
        if (br == 1200 || br == 2400 || br == 4800 || br == 9600){
          EEPROM.put(EEP_BR, br);
          reset(60E3);
        }
      }
      if (cmd.equals("ns")){
        unsigned int ns = (entry.substring(5,entry.length())).toInt();
        maxSynchs = ns;
        EEPROM.write(EEP_NS, maxSynchs);
      }
      if (cmd.equals("in")){
        initialize();
        reset(60E3);
      }
    }
  }
  
  else{
    Serial.println("Invalid Command");
  }
}

/*
 * 
 */
void initialize(){
  EEPROM.put(EEP_DIST,DEF_DI);
  EEPROM.write(EEP_TT, DEF_TT);
  EEPROM.put(EEP_DP, DEF_DP);
  EEPROM.put(EEP_PR, DEF_PR);
  EEPROM.put(EEP_DE, DEF_DE);
  EEPROM.put(EEP_BR, DEF_BR);
  EEPROM.write(EEP_FT, DEF_FT);
  EEPROM.write(EEP_GL, DEF_GL);
  EEPROM.write(EEP_NS,DEF_NS);
  EEPROM.put(EEP_SE,DEF_SE);
}

/*
 * Send a string command to a single node
 */
void sendCommand(int n, String cmd){
  if (Serial) Serial.println(cmd);
  int len = cmd.length() - 1;
  if (len >= CMD_LENGTH) return;
  char tmp[len];
  cmd.remove(0,2);
  strncpy(tmp, cmd.c_str(), len);
  tmp[len - 1] = 0;
  for (int i = 0; i < len; i++) nodes[n].opCmd[i] = tmp[i];
  nodes[n].opCmdIndex = 0;   
}

/*
 * 
 */
void loadParameters(){
  distance = readEEPROM(EEP_DIST);
  timerTop = EEPROM.read(EEP_TT);
  freq = 16E6/((timerTop+1)*LOOKUP_POINTS);
  numDataPoints = readEEPROM(EEP_DP);
  pollingRate = readEEPROM(EEP_PR);
  depth = readEEPROM(EEP_DE);
  baudRate = readEEPROM(EEP_BR);
  firstTransmitter = EEPROM.read(EEP_FT);
  gainLevel = EEPROM.read(EEP_GL);
  maxSynchs = EEPROM.read(EEP_NS);
  binSequence = readEEPROM(EEP_SE);
  year = EEPROM.read(EEP_YR);
  month = EEPROM.read(EEP_MO);
  day = EEPROM.read(EEP_DA);
  hour = EEPROM.read(EEP_HR);
  minute = EEPROM.read(EEP_MN);
}

/*
 * 
 */
 void wait(unsigned long t){
  unsigned long startTime = millis();
  while ((millis() - startTime) < t){};
 }

 /*
 * 
 */
String createFile(){
  if (control & (1 << SD_WRITE)){
    String folder = String(date) + "/";
    String fileName =  String(tm) + ".m";
    if (!SD.exists(folder)) SD.mkdir(folder);
    int append = 0;    
    while((append < 10) && SD.exists(folder+fileName)){
      fileName = String(tm) + "_" + String(append) + ".m";
      append++;
    }
    File dataFile = SD.open(folder + fileName, FILE_WRITE);   
    dataFile.close();
    return folder + fileName;
  }
  else return "";
}

/*
 * 
 */
 void printData(String dataString){
    if (Serial) Serial.print(dataString);
    
    if ((currentFile != "") && (control & (1 << SD_WRITE))){
      File dataFile = SD.open(currentFile, FILE_WRITE);
      if (dataFile){
        dataFile.print(dataString);
        dataFile.close();
        //Flash the SD Lamp on successful write
        PORTB ^= (1 << PB6);      
      }
    }
 }

 /*
  * 
  */
  void updateDT(int increment){
    second += increment;       
    if (second >= 60){
      second = second%60;
      minute++;
      if (minute < 60) EEPROM.write(EEP_MN,minute);
      else{
        minute = 0;
        EEPROM.write(EEP_MN,minute);
        hour++;
        if (hour < 24) EEPROM.write(EEP_HR,hour);
        else{
          hour = 0;
          EEPROM.write(EEP_HR,hour);
          day++;
          if (day < 32) EEPROM.write(EEP_DA,day);
          else{
            day = 1;
            EEPROM.write(EEP_DA,day);
            month++;
            if (month < 13) EEPROM.write(EEP_MO,month);
            else{
              month = 1; 
              EEPROM.write(EEP_MO,month);
              year++;
              EEPROM.write(EEP_YR,year);
            }
          }
        }
      }    
    }
    date[0] = '\0';
    tm[0] = '\0';
    sprintf(date, "20%02d%02d%02d", year, month, day);
    sprintf(tm, "%02d%02d%02d", hour, minute, second);
  }

