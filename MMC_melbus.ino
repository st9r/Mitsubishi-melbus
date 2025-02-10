


#define INT_NUM (byte)0         //Interrupt number (0/1 on ATMega 328P)
#define MELBUS_CLOCKBIT (byte)2 //Pin D2  - CLK
#define MELBUS_DATA (byte)3     //Pin D3  - Data
#define MELBUS_BUSY (byte)4     //Pin D4  - Busy


const byte play = 7;
const byte MISC = 6;         //A6


byte track = 0x01; //Display show HEX value, not DEC. (A-F not "allowed")
byte cd = 0x01; //1-10 is allowed (in HEX. 0A-0F and 1A-1F is not allowed)


//volatile variables used inside AND outside of ISP
volatile byte melbus_ReceivedByte = 0;
volatile byte melbus_Bitposition = 7;
volatile bool byteIsRead = false;


byte byteToSend = 0;          //global to avoid unnecessary overheadall
bool reqMasterFlag = false;   //set this to request master mode (and sendtext) at a proper time.

#define CDC_RESPONSE_ID 0xEB 
#define CDC_MASTER_ID 0xEF 
#define CDC_init_ID 0xEE

#define CDC_BASE_ID 0xE8 
#define CDC_ALT_ID 0xE9 



//Defining the commands. First byte is the length of the command.
#define MRB_1 {3, 0x00, 0x42, 0xEC}            //Master Request Broadcast version 1
#define MRB_2 {3, 0x00, 0x43, 0xEC}            //Master Request Broadcast version 2 (maybe this is second init seq?)
#define MRB_3 {3, 0x00, 0x46, 0xEC}            //Master request Broadcast version 3
#define MI {3, 0x07, 0x42, 0xEE}               //Main init sequence
#define SI {3, 0x00, 0x46, 0xED}               //Secondary init sequence (turn off ignition, then on)


#define IGN_OFF {3, 0x00, 0x40, 0x12}           //this is the last message before HU goes to Nirvana ok



#define CDC_CIR {3, CDC_BASE_ID, 0x46, 0xEF}             //Cartridge info request. Respond with 6 bytes

#define CDC_TIR {5, CDC_ALT_ID, 0x43, 0xE0, 0x01, 0x08}  //track info req. resp 9 bytes ///or43 //53// //or 41
// #define CDC_TIR {5, CDC_ALT_ID, 0x41, 0xE0, 0x01, 0x08}  //track inf
// #define CDC_TIR {5, CDC_ALT_ID, 0x53, 0xE0, 0x01, 0x08}  //track inf

#define CDC_NXT {5, CDC_BASE_ID, 0x46, 0x2D, 0x40, 0x01} //next track. 1b>46 //43 ef 0 3 6 
#define CDC_PRV {5, CDC_BASE_ID, 0x46, 0x2D, 0x00, 0x01} //prev track ok

#define CDC_CHG {3, CDC_BASE_ID, 0x43, 0x50}             //change cd     1a > 43

#define CDC_PUP {3, CDC_BASE_ID, 0x41, 0x2F}             //power up. resp ack (0x00).
#define CDC_PDN {3, CDC_BASE_ID, 0x41, 0x22}             //power down. ack (0x00)

#define CDC_FFW {3, CDC_BASE_ID, 0x41, 0x29}             //FFW. ack
#define CDC_FRW {3, CDC_BASE_ID, 0x41, 0x26}             //FRW. ack
#define CDC_SCN {3, CDC_BASE_ID, 0x43, 0x2E}       ///or 0x41, 2E      //scan mode. ack
#define CDC_RND {3, CDC_BASE_ID, 0x41, 0x52}             //random mode. ack
#define CDC_NU {3, CDC_BASE_ID, 0x1A, 0x50}              //not used
      //master req broadcast. wait for MASTER_ID and respond with MASTER_ID.


enum {
  E_MRB_1,  
  E_MI,       
  E_SI,     
  E_MRB_2,    
  E_MRB_3,
  E_IGN_OFF,  
  E_CDC_CIR,  
  E_CDC_TIR,  
  E_CDC_NXT,   
  E_CDC_PRV,  
  E_CDC_CHG,  
  E_CDC_PUP,   
  E_CDC_PDN,
  E_LIST_MAX  
};

const byte commands[][8] = {
  MRB_1,  
  MI,     
  SI,    
  MRB_2, 
  MRB_3,
  IGN_OFF, 
  CDC_CIR,  
  CDC_TIR, 
  CDC_NXT, 
  CDC_PRV, 
  CDC_CHG, 
  CDC_PUP, 
  CDC_PDN, 

  
};

enum {
  STATE_INVALID,
  STATE_CDC,

};

const byte listLen = E_LIST_MAX; //how many rows in the above array


byte current_state = 0;
// some CDC (CD-CHANGER) data
byte startByte = 0x08; //on powerup - change trackInfo[1] & [8] to this
byte stopByte = 0x02; //same on powerdown
byte cdcTrackInfo[] =     {0x00, 0x02, 0x00, 0x06, 0x80, 0x03, 0x0C, 0xCC, 0xCC, 0x0C, 0xCC}; ///del 1 oo
byte cdcCartridgeInfo[] = {0x80, 0x00, 0x0F, 0x06, 0x00, 0x0F};//{0xE8, 0x43, 0xEF, 0x00, 0x03, 0x06};

/*
 *      **** SETUP ****
*/


void setup() {
  //Disable timer0 interrupt. It's is only bogging down the system. We need speed!
  TIMSK0 &= ~_BV(TOIE0);

  //All lines are idle HIGH
  pinMode(MELBUS_DATA, INPUT_PULLUP);
  pinMode(MELBUS_CLOCKBIT, INPUT_PULLUP);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);
  pinMode(MISC, OUTPUT);
                digitalWrite(MISC, LOW);

Serial.begin(38400, SERIAL_8N1);
  //Activate interrupt on clock pin
  attachInterrupt(digitalPinToInterrupt(MELBUS_CLOCKBIT), MELBUS_CLOCK_INTERRUPT, RISING);
  //Call function that tells HU that we want to register a new device
  melbusInitReq();
}

/*        **************************************************
          MAIN LOOP
          **************************************************
*/


void loop() {
  static byte lastByte = 0;     //used to copy volatile byte to register variable. See below
  static long runOnce = 300000;     //counts down on every received message from HU. Triggers when it is passing 1.
  static long runPeriodically = 100000; //same as runOnce but resets after each countdown.
  static bool powerOn = true;
  static long HWTicks = 0;      //age since last BUSY switch
  static long ComTicks = 0;     //age since last received byte
  static long ConnTicks = 0;    //age since last message to SIRIUS SAT
  static long timeout = 1000000; //should be around 10-20 secs
  static byte matching[listLen];     //Keep track of every matching byte in the commands array
  //static char text_array[17] = {0};

  //these variables are reset every loop
  byte byteCounter = 1;  //keep track of how many bytes is sent in current command
  byte melbus_log[99];  //main init sequence is 61 bytes long...
  bool BUSY = PIND & (1 << MELBUS_BUSY);

  HWTicks++;
  if (powerOn) {
    ComTicks++;
    ConnTicks++;
  } else {
    ComTicks = 1;
    ConnTicks = 1;
    //to avoid a lot of serial.prints when its 0
  }

  //check BUSY line active (active low)
  while (!BUSY) {
    HWTicks = 0;  //reset age

    //Transmission handling here...
    if (byteIsRead) {
      byteIsRead = false;
      lastByte = melbus_ReceivedByte; //copy volatile byte to register variable
      ComTicks = 0; //reset age
      melbus_log[byteCounter - 1] = lastByte;
      //Loop though every command in the array and check for matches. (No, don't go looking for matches now)
      for (byte cmd = 0; cmd < listLen; cmd++) {
        //check if this byte is matching
        if (lastByte == commands[cmd][byteCounter]) {
          matching[cmd]++;
          //check if a complete command is received, and take appropriate action
          if ((matching[cmd] == commands[cmd][0]) && (byteCounter == commands[cmd][0])) {
            byte cnt = 0;
            byte b1 = 0, b2 = 0;
            ConnTicks = 0;  //reset age

            switch (cmd) {

              case E_SI:
              case E_MI:

                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                   
                    if (melbus_ReceivedByte == CDC_init_ID) {
                      byteToSend = CDC_RESPONSE_ID;
                      SendByteToMelbus();
                    }
                   
                  }
                }
                digitalWrite(MISC, HIGH);
                //Serial.println("main init");
                break;
              
              case E_CDC_CIR:
                SendCartridgeInfo(cdcCartridgeInfo);
                break;
              case E_CDC_TIR:
                SendTrackInfo(cdcTrackInfo);
                break;
                case E_CDC_NXT:
                track = fixTrack(++track);
                 Serial.println("N");
                cdcTrackInfo[5] = track;
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
          
              case E_CDC_PRV:
                //track = fixTrack(--track);
                cdcTrackInfo[5] = track;
                 Serial.println("P");
               
                byteToSend = 0x00;
                SendByteToMelbus();
                break;
           
              case E_CDC_CHG:
              while (!(PIND & (1 << MELBUS_BUSY))) {
                if (byteIsRead) {
                    byteIsRead = false;
                   // changeCD(&cd, &track, melbus_ReceivedByte);
                    byteToSend = 0x00;
                    SendByteToMelbus();
                  }
                }
                if(cd > 6) {
                  cd = 1;
                }
                if (current_state == STATE_CDC) {
                  if (cd < 1) {
                    cd = 6;
                  }
                } 
                cdcTrackInfo[5] = track;
                cdcTrackInfo[3] = cd;
                break;
              case E_CDC_PUP:
                byteToSend = 0x00;
                SendByteToMelbus();
                cdcTrackInfo[1] = startByte;
                cdcTrackInfo[8] = startByte;
                digitalWrite(play, HIGH);
                digitalWrite(MISC, HIGH);

                cd = 1;
                track = 1;
                current_state = STATE_CDC;
                break;
              case E_CDC_PDN:
                byteToSend = 0x00;
                SendByteToMelbus();
                cdcTrackInfo[1] = stopByte;
                cdcTrackInfo[8] = stopByte;
                digitalWrite(play, LOW);
                digitalWrite(MISC, LOW);
                break;

              case E_IGN_OFF:
              digitalWrite(MISC, LOW);

                break;
             
            } //end switch
            break;    //bail for loop. (Not meaningful to search more commands if one is already found)
          } //end if command found
        } //end if lastbyte matches
      }  //end for cmd loop
      byteCounter++;
    }  //end if byteisread
    //Update status of BUSY line, so we don't end up in an infinite while-loop.
    BUSY = PIND & (1 << MELBUS_BUSY);
  }


  //Do other stuff here if you want. MELBUS lines are free now. BUSY = IDLE (HIGH)
  //Don't take too much time though, since BUSY might go active anytime, and then we'd better be ready to receive.

  //Printing transmission log (from HU, excluding our answer and things after it)
  // if (ComTicks == 0) {                    //print all messages
  // //if (ComTicks == 0 && ConnTicks != 0) {    //print unmatched messages (unknown)
  //   for (byte b = 0; b < byteCounter - 1; b++) {
  //     Serial.print(melbus_log[b], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.println();
  // }

  //runOnce is counting down to zero and stays there
  //after that, runPeriodically is counting down over and over...
  if (runOnce >= 1) {
    runOnce--;
  } else if (runPeriodically > 0) runPeriodically--;


  //check if BUSY-line is alive
  if (HWTicks > timeout) {
  //  Serial.println("BUSY line problem");
    HWTicks = 0;
    ComTicks = 0;     //to avoid several init requests at once
    ConnTicks = 0;    //to avoid several init requests at once
    //while (1);      //maybe do a reset here, after a delay.
    melbusInitReq();  //worth a try...
  }

  //check if we are receiving any data
  if (ComTicks > timeout) {
    //Serial.println("COM failure (check CLK line)");
    ComTicks = 0;
    ConnTicks = 0;    //to avoid several init requests at once
    melbusInitReq();  //what else to do...
  }

  //check if HU is talking to us specifically, otherwise force it.
  if (ConnTicks > timeout) {
   // Serial.println("Lost connection. Re-initializing");
    ConnTicks = 0;
    melbusInitReq();
  }


  //Reset stuff
  melbus_Bitposition = 7;
  for (byte i = 0; i < listLen; i++) {
    matching[i] = 0;
  }



}



/*
                END LOOP
*/

//Notify HU that we want to trigger the first initiate procedure to add a new device
//(CD-CHGR/SAT etc) by pulling BUSY line low for 1s
void melbusInitReq() {
  //Serial.println("conn");
  //Disable interrupt on INT_NUM quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT_NUM);

  // Wait until Busy-line goes high (not busy) before we pull BUSY low to request init
  while (digitalRead(MELBUS_BUSY) == LOW) {}
  delayMicroseconds(20);

  pinMode(MELBUS_BUSY, OUTPUT);
  digitalWrite(MELBUS_BUSY, LOW);
  //timer0 is off so we have to do a trick here
  for (unsigned int i = 0; i < 12000; i++) delayMicroseconds(100);

  digitalWrite(MELBUS_BUSY, HIGH);
  pinMode(MELBUS_BUSY, INPUT_PULLUP);
  //Enable interrupt on INT_NUM, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT_NUM);
}


//This is a function that sends a byte to the HU - (not using interrupts)
//SET byteToSend variable before calling this!!
void SendByteToMelbus() {
  //Disable interrupt on INT_NUM quicker than: detachInterrupt(MELBUS_CLOCKBIT_INT);
  EIMSK &= ~(1 << INT_NUM);

  //Convert datapin to output
  //pinMode(MELBUS_DATA, OUTPUT); //To slow, use DDRD instead:
  DDRD |= (1 << MELBUS_DATA);

  //For each bit in the byte
  for (char i = 7; i >= 0; i--)
  {
    while (PIND & (1 << MELBUS_CLOCKBIT)) {} //wait for low clock
    //If bit [i] is "1" - make datapin high
    if (byteToSend & (1 << i)) {
      PORTD |= (1 << MELBUS_DATA);
    }
    //if bit [i] is "0" - make datapin low
    else {
      PORTD &= ~(1 << MELBUS_DATA);
    }
    while (!(PIND & (1 << MELBUS_CLOCKBIT))) {}  //wait for high clock
  }
  //Let the value be read by the HU
  delayMicroseconds(20);
  //Reset datapin to high and return it to an input
  //pinMode(MELBUS_DATA, INPUT_PULLUP);
  PORTD |= 1 << MELBUS_DATA;
  DDRD &= ~(1 << MELBUS_DATA);

  //We have triggered the interrupt but we don't want to read any bits, so clear the flag
  EIFR |= 1 << INT_NUM;
  //Enable interrupt on INT_NUM, quicker than: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
  EIMSK |= (1 << INT_NUM);
}


//This method generates our own clock. Used when in master mode.
void SendByteToMelbus2() {
  delayMicroseconds(700);
  //For each bit in the byte
  //char, since it will go negative. byte 0..255, char -128..127
  //int takes more clockcycles to update on a 8-bit CPU.
  for (char i = 7; i >= 0; i--)
  {
    delayMicroseconds(7);
    PORTD &= ~(1 << MELBUS_CLOCKBIT);  //clock -> low
    //If bit [i] is "1" - make datapin high
    if (byteToSend & (1 << i)) {
      PORTD |= (1 << MELBUS_DATA);
    }
    //if bit [i] is "0" - make datapin low
    else {
      PORTD &= ~(1 << MELBUS_DATA);
    }
    //wait for output to settle
    delayMicroseconds(5);
    PORTD |= (1 << MELBUS_CLOCKBIT);   //clock -> high
    //wait for HU to read the bit
  }
  delayMicroseconds(20);
}

//This method generates our own clock AND drives busy low. Used in master mode in MD-CHGR mode
void SendByteToMelbus3() {
  delayMicroseconds(527);
  for (char i = 7; i >= 0; i--) {
    PORTD &= ~(1 << MELBUS_CLOCKBIT);  //clock -> low
    delayMicroseconds(8);
    if (byteToSend & (1 << i)) {
      PORTD |= (1 << MELBUS_DATA);
    } else {
      PORTD &= ~(1 << MELBUS_DATA);
    }
    PORTD |= (1 << MELBUS_CLOCKBIT);   //clock -> high
    //wait for output to settle
    delayMicroseconds(8);
    //wait for HU to read the bit
  }
  delayMicroseconds(20);
}

//Global external interrupt that triggers when clock pin goes high after it has been low for a short time => time to read datapin
void MELBUS_CLOCK_INTERRUPT() {
  //Read status of Datapin and set status of current bit in recv_byte
  //if (digitalRead(MELBUS_DATA) == HIGH) {
  if ((PIND & (1 << MELBUS_DATA))) {
    melbus_ReceivedByte |= (1 << melbus_Bitposition); //set bit nr [melbus_Bitposition] to "1"
  }
  else {
    melbus_ReceivedByte &= ~(1 << melbus_Bitposition); //set bit nr [melbus_Bitposition] to "0"
  }

  //if all the bits in the byte are read:
  if (melbus_Bitposition == 0) {
    //set bool to true to evaluate the bytes in main loop
    byteIsRead = true;

    //Reset bitcount to first bit in byte
    melbus_Bitposition = 7;
  }
  else {
    //set bitnumber to address of next bit in byte
    melbus_Bitposition--;
  }
}




void reqMaster() {
  DDRD |= (1 << MELBUS_DATA); //output
  PORTD &= ~(1 << MELBUS_DATA);//low
  delayMicroseconds(700);
  delayMicroseconds(800);
  delayMicroseconds(800);
  PORTD |= (1 << MELBUS_DATA);//high
  DDRD &= ~(1 << MELBUS_DATA); //back to input_pullup
}





byte fixTrack(byte track) {
  //cut out A-F in each nibble, and skip "00"
  byte hn = track >> 4; // high nibble
  byte ln = track & 0xF; // low nibble
  if (ln == 0xA) {
    ln = 0;
    hn += 1;
  }
  if (ln == 0xF) {
    ln = 9;
  }
  if (hn == 0xA) {
    hn = 0;
    ln = 1;
  }
  if ((hn == 0) && (ln == 0)) {
    ln = 0x9;
    hn = 0x9;
  }
  return ((hn << 4) + ln);
}
 

void SendTrackInfo(byte trackInfo2[]) {
  for (byte i = 0; i < 9; i++) {
    byteToSend = trackInfo2[i];
    SendByteToMelbus();
  }
}

void SendCartridgeInfo(byte cartridgeInfo2[]) {
  for (byte i = 0; i < 6; i++) {
    byteToSend = cartridgeInfo2[i];
    SendByteToMelbus();
  }
}
