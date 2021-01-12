//Changable Information
const unsigned short baudrate = 250;//baudrate in thousands
const bool high_noise_tolerance = true;//Set to false to make bit capture timing more strict
const bool use_rpm = true;//Set to false to allow new addresses to be claimed after engine is running
const bool name_match = false;//Set to false to stop all address claims on bus while engine is running, *IF BOTH FALSE NO ADDRESS CLAIMS WILL BE PREVENTED*
const unsigned short engine_indicator = 13;//Pin used for LED showing if engine is on
const unsigned short RX = 23;//GPIO pin for receiving CAN messages
const unsigned short TX = 22;//GPIO pin to implement BitBanging technique on CAN line

//Global Variable Information
unsigned long ID = 0;//CAN message ID
unsigned long data1 = 0;//First half of data field
unsigned long data2 = 0;//Second half of data field
unsigned short bitpos = 0;//Bit Position
unsigned short temp_numBit = 0;//Used for samples taken without edge
unsigned short numBit = 0;//Used for calculations while keeping true numBits saved
volatile unsigned long Tdeltas[130] = {};//Timestamps gathered, Maximum amount of changes with 10 added for buffer
bool bitlevel = 0;//Bit value of samples
volatile unsigned short pending = 0;//Bits waiting to be processed
volatile unsigned short processed = 0;//Bits processed
unsigned short tolerance = 0;//Changes bit capture tolerance
const unsigned short bitwidth = (1000 / baudrate);//Calculated Bitwidth in us depending on CAN baudrate
const unsigned short eofwidth = (11 * bitwidth) - (bitwidth / 2);//End of Frame will reset variables used for calculations
unsigned short noise = 0;

// Targeting
unsigned long *target = NULL;//Keeps track of message sections to construct
bool engine_on = false;//Used to keep track of when vehicle engine is running
bool skip_msg = true;//Used to skip unneccessary message construction
unsigned long s_addr[255][2]={};//Array of saved address claim names
bool claimed[255] = {};//Used to ensure addresses cannot be overwritten once claimed
bool claim_indicator = false;//Checks if data in address claim matches associated source address


void setup() {
    // put your setup code here, to run once:
    
  //Creates tolerance based on noise/resolution error 
  for (int i = 0; i < 255; i++){ claimed[i] = false; }
  if (high_noise_tolerance == false){tolerance = 1;}
  noise = (bitwidth / 2 + tolerance);
  
  //Pins for ISR and BitBanging
  pinMode(RX,INPUT);// sets the RX as input to read CAN traffic
  pinMode(TX,OUTPUT);//sets the TX as output to employ bitbang as needed
  attachInterrupt(digitalPinToInterrupt(RX),ISR_CAN,CHANGE);//Interrupt to detect edges of bit changes
}

void loop() {

// ------------------------------------------------------- Initial Checks -------------------------------------------------------------

  if (skip_msg == true || pending <= processed){return;}

  //------------------------------------------------------ TEMPS ----------------------------------------------------------------------

  unsigned short temp_processed = processed;

  if(temp_processed == 0){                                                          //Reset of variables for new message construction
    ID = 0;
    data1 = 0;
    data2 = 0;
    bitpos = 0;
    bitlevel = 1;
    target = &ID;
  }
    
  if(temp_processed != processed){                                                  //Checks to ensure no new message has started
    return;
  }

  //------------------------------------------------------ Numbit, Bitlevel Calculation -------------------------------------------------
  
  bitlevel =! bitlevel;
  bool reduce = false;

  if(temp_numBit == 5 || temp_processed == 0){reduce = true;}                        //Subtracts stuff bits and SOF, these are not included in calculated bitpos
       
  temp_numBit=Tdeltas[temp_processed] / bitwidth;                                    //Rounding method to ensure correct bits are calculated if delta of edges are not perfectly divisible by bitwidth
  if((Tdeltas[temp_processed] % bitwidth) >= (bitwidth / 2)){temp_numBit++;}
  
  if(reduce == true){numBit = temp_numBit -1;}                                       //numBit is applied, keeping prev_numBit true, in case 2 stuffbits need to be calculated in succession
  
  else{numBit = temp_numBit;}

  //------------------------------------------------------ Numbit processing  -----------------------------------------------------------

   for(unsigned short j = 0; j < numBit; j++){      
    bitpos++;

    if(ID == 0x0CF00400 && use_rpm == true){                                         //Message containing rpm data bytes
      if(bitpos < 63 && engine_on == true){
        engine_on = false;
        digitalWrite(engine_indicator,LOW);
      }

     if(bitpos >= 63 && bitpos <= 78){
       if(bitlevel == 1 && engine_on == false){
         engine_on = true;
         digitalWrite(engine_indicator,HIGH);
        }
      }
    }
    
    if(bitpos == 32){                                                                //Only need to calculate rpm and address claim messages
      if(ID&0x00FF0000 != 0x00EE0000 || ID&0x00FFFF00 != 0x00F00400){
        skip_msg = true; return; }
    }
      else if(bitpos == 39){target = &data1;}                                        //First 32 bits of data field

    else if(bitpos == 71){target = &data2;}                                          //Second 32 bits of data field
    
    if(bitpos != 12 && bitpos != 13 &&! (bitpos>31 && bitpos<39) &&! (bitpos > 102)){//Only calculates target for applicable bitpos'
         *(target) = (*(target) << 1) | bitlevel;
    }
   }   
    
  // ----------------------------------------------------- Attack Detection -------------------------------------------------------------
   
  if((ID&0x00FF0000) == 0x00EE0000 && bitpos > 102){//Address Claim name saving at first received
    
    if(name_match == true && (engine_on == false && claimed[ID&0x000000FF] == false)){
        s_addr[ID&0x000000FF][0] = data1;
        s_addr[ID&0x000000FF][1] = data2;
        claimed[ID&0x000000FF] = true;
        skip_msg = true; 
    }
    
    if(name_match == true && (s_addr[ID&0x000000FF][0] != data1 || s_addr[ID&0x000000FF][1] != data2)){
      claim_indicator = true;
    }
    else{ claim_indicator == false; }

    if((use_rpm == false && claim_indicator == true) || (engine_on == true && name_match == false) || (engine_on == true && claim_indicator == true)){//Attack Parameters and BitBang
        digitalWrite(TX,HIGH);
        digitalWrite(TX,LOW);
        skip_msg = true;
    }
  }

  // ----------------------------------------------------- TEMP to GLOBAL ---------------------------------------------------------------
    temp_processed++;
    if (temp_processed != (processed + 1)){return;}                                   // If ISR changes globals, indicating new message has started transmitting
        
    else{processed = temp_processed;}
 
}   

  // ----------------------------------------------------- Interrupt Service Routine ----------------------------------------------------
  
void ISR_CAN(){                                                                       
  static elapsedMicros dT;
  if (dT >= noise){
    if(dT >= eofwidth){
      pending = 0;
      processed = 0;
      skip_msg = false;
    }
    else{
      Tdeltas[pending] = dT;
      pending++;    
    }
  }
  dT = 0;
}
