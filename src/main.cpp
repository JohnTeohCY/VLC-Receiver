#include <Arduino.h>
#include <SPI.h>
#include <stdlib.h>
#include <stdio.h>

#define SAMPPERIOD 50            /* Sampling period in micro-seconds */
#define SAMPPERBIT 4
#define TIMER_COUNT 150         /* TIMER_COUNT is 150 for 150MHz ipg_clk_root */
#define BUFLEN 28                /* BUFLEN = 4 * Starting sequence */
#define ARROWLIM 27              /* Arrow Limit = BUFLEN - 1 */
#define DATALEN 16368
#define DUMLEN 16

/* Pins */
const int PDpin = 15;          // Analog pin A1 
// const int CS = 10;          // SPI CS 
// const int MISOpin = 12;     // SPI MISOpin
// const int SCLKpin = 13;     // SPI SCLKpin 

/* Const Variables */
const float Vref = 3.29;
const float fullScaleCode = 255.0;
const float thresWeight = 1.8;
const float defaultThres = 0.35;
const float minNoisy = 0.15;
const char zero = '0';
const char one = '1';
const unsigned int mask = 17895697;
const unsigned int pattern = 17891344;

/* State Machine */
enum FSMState { TIME, SEARCH, RECORD, DECODE};
volatile FSMState state;
volatile int stage;

/* Global Variables */
volatile unsigned int result = 0;
volatile float result_f = 0.0;
volatile float locResult = 0.0;
volatile float noiseFloor = 0.0;
volatile float peakMag = 0.0;
volatile float threshold = 0.5;
volatile int modScheme = 0;
volatile unsigned int codeword = 0;
volatile unsigned int message = 0;
volatile unsigned int message8bit = 0;

// Flags
volatile boolean startDetection = 0;
volatile boolean bitSampled = 0;
volatile boolean detected = 0;
volatile boolean confirmed = 0;

// Counters
volatile int sampleCount = 0;
volatile int chipCount = 0;
volatile int codewordCount = 0;
volatile int messageCount = 0;

// Utility
uint32_t startTime, elapsedTime;

// Arrows
// volatile int curId = 0;
volatile int s1 = 0;
// volatile int s2 = 24;
// volatile int s3 = 20;
// volatile int s4 = 16;
// volatile int s5 = 12;
// volatile int s6 = 8;
// volatile int s7 = 4;

// Arrays
volatile unsigned int searchBitArr = 0;
volatile unsigned int detection = 0;
float searchArr[BUFLEN] = {0.0};       // Analog front-end to determine noise floor and maximum signal amplitude
// boolean boolSearchArr[BUFLEN] = {0};   // Digital back-end to identify starting sequence
boolean modArr[4] = {0};               // Able to accomodate 16 different modulation scheme (GAVE UP ON THE 4 HAMMING DISTANCE)
// boolean dataArr[16] = {0};             // Largest codeword is 16-chips wide
char dataStr[100] = "";

// const float dummyArr[DUMLEN] = {0.02, 0.03, 0.02, 0.02, 0.1, 0.4, 0.5, 0.53, 0.52, 0.52, 0.52, 0.52, 0.5, 0.1, 0.04, 0.02};// Dummy starting sequence
// volatile int dummyCounter = 0;


/* Function Declarations */
static void sampChannel(void);
static void decodeModScheme(void);
static void decodeData(void);
static void reset(void);

void setup() {
  
   CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL; // Using 150MHz ipg_clk_root

   /* ROUTE CLOCK TO GPTs */
   CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON); // Enable clock to GPT1 module
   CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON); // Enable clock to GPT2 module

   /* INIT */
   cli();

   GPT1_CR = 0;                            // Disable for configuration
   GPT1_PR = 1 - 1;                        // Prescale GPT1 source clock by 1 => 24 MHz or 150 MHz depending on source clock set by CCM_CSCMR1
   GPT1_CR = GPT_CR_CLKSRC(1)              /* 150 MHz peripheral clock as clock source */         
            & ~GPT_CR_FRR                 /* Restart counter upon compare event */;             
   // | GPT_CR_FRR /* Free-Run, do not reset */;                                                   
   GPT1_OCR1 = (SAMPPERIOD*TIMER_COUNT)-1; // set the value for when a compare event is generated = (req period / timer period) - 1
   GPT1_IR = GPT_IR_OF1IE;                 // enable GPT interrupts, write 1s to their enable bits in IR
   // Enable Interrupt
   NVIC_ENABLE_IRQ(GPT1_SR&(GPT_SR_OF1));
   attachInterruptVector(IRQ_GPT1, &sampChannel);
   NVIC_SET_PRIORITY(IRQ_GPT1, 0); // Top priority, lower number higher priority
   NVIC_ENABLE_IRQ(IRQ_GPT1);

   sei();

   // Set ADC resolution to 8 bits and no averaging (for sampling interval of approx 3-4 us)
   analogReadRes(8);
   analogReadAveraging(1);
  
   while (!Serial) {}
   
   /*
   pinMode(CS, OUTPUT);
   pinMode(MISOpin, OUTPUT);
   pinMode(SCLKpin, OUTPUT);

   digitalWriteFast(CS, HIGH); // CS is active HIGH

   SPI.begin();
   */

   Serial.println("Enter s to start detecting...");

   /* Enable Timer */
  //  GPT1_CR |= GPT_CR_EN;

   state = TIME;
}

FASTRUN void loop() {
  
  switch (state) {
   case TIME:
      if (Serial.available() > 0) { // If user has input command
        char receivedChar = Serial.read(); // 1 char in ASCII = 1 byte
        switch (receivedChar) {
          case 's':
            startDetection = 1;
            /* Enable Timer */
            GPT1_CR |= GPT_CR_EN;
            break;
          case 'o':
            cli();
            startDetection = 0;
            /* Disable Timer */
            GPT1_CR &= ~GPT_CR_EN;
            Serial.println("OFF");
            // reset everything
            reset();
            sei();
            break;
          default:
            break;
        }
      }

      if (startDetection) {
        // Wait for sampChannel's bitSampled, then transition to other states
        if (bitSampled && !confirmed) {
          bitSampled = 0;
          state = SEARCH;
        }
        else if (bitSampled && confirmed) {
          bitSampled = 0;
          state = RECORD;
        }
      }

      break;

   case SEARCH:
      cli();
      // Convert to Voltage
      // result = result >> 4;
      result_f = (float) result  * Vref / fullScaleCode;
      // Serial.println(result_f, 2);

      // Populate the searchArr
      searchArr[s1] = result_f;

      /* Dynamic thresholding algorithm */ 
      //  - reset noiseFloor and peakMag upon entering
      //  - search for min in searchArr, call that the noise floor.
      //  - search for max in searchArr, call that the peak magnitude.
      //  - check if peak magnitude is at least 2 times noise floor, if yes, set threshold to midpoint.
      //  - check if noise floor is less than 0.15. If it is, auto set threshold to 0.35
      //  - if noise floor is non-zero, and prev conditions not met, threshold is 1.8 * noise floor
      noiseFloor = 3.3;
      peakMag = 0.0;
      for (int i = 1; i < BUFLEN; i++) {
        if (searchArr[i] < noiseFloor) {
            noiseFloor = searchArr[i];
        }
        if (searchArr[i] > peakMag) {
            peakMag = searchArr[i];
        }
      }

      // Use addition instead of multiplicative factor
      if (peakMag < noiseFloor + 0.05) { // If there is no clear peak
        threshold = noiseFloor + 0.05;
      } else {
        threshold = (noiseFloor + peakMag)/2;
      }
      
      // Do thresholding and populate the boolSearchArr
      searchBitArr = searchBitArr << 1;
      if (result_f > threshold) {
         searchBitArr = searchBitArr | 1;
      } else {
         searchBitArr = searchBitArr | 0;
      }

      // Check array elems at arrow positions
      // 7 bit Barker Code: 1  1  1  0  0  1  0
      // Arrow positions:   S7 S6 S5 S4 S3 S2 S1
      detection = searchBitArr & mask;
      if (detection == pattern) {
        if (!detected) {
          detected = 1;
        } else {
          
          //Serial.println(threshold);
          /* for MATLAB: print the whole searchArr ------------------------------------------------------------------
          Serial.println(threshold);
          Serial.println(s1);
          for (int i = 1; i < BUFLEN; i++) {
            Serial.print(searchArr[i], 3);
            Serial.print(',');
          }
          Serial.println(' ');
          
          // -- end -- */

          confirmed = 1;
          searchBitArr = 0;
          detection = 0;
          //memset(boolSearchArr, 0, sizeof(boolSearchArr));
          memset(searchArr, 0, sizeof(searchArr));
          //Serial.println(threshold);
          //Serial.println(peakMag);
          //Serial.println(noiseFloor);
        }
      }

      // Increment arrow positions
      if (s1 == ARROWLIM) {
         s1 = 0;
      } else {
         s1++;
      }

      // Change states
      state = TIME;
      if (confirmed) {
         state = RECORD;
         detected = 0;
         // confirmed = 0;
         sampleCount = 0;
      }

      sei();
      break;

   case RECORD:
      /* 3 STAGES */
      // 1: sample noise floor
      // 2: sample modulation scheme
      // 3: sample user data
      // no cli() because it doesn't matter if sampChannel() interrupts midway

      if (sampleCount == SAMPPERBIT) { // Only enters every 4th sample (CHIPPERIOD)
         cli();
         sampleCount = 0;
         // Convert to Voltage
         // result = result >> 4;
         result_f = (float) result  * Vref / fullScaleCode;
         sei();

         // For MATLAB: print result_f --------------------------------------------------------------------------------
         //Serial.print(result_f, 3);
         //Serial.print(',');
         // -- end -- */
         

         switch (stage) {
            case 0:
               // Readjust threshold value if needed (TOO HARD?)
               noiseFloor += result_f;
               chipCount++;

               if (chipCount == 4) {                        // TO DO: replace 2 with const int
                  noiseFloor = noiseFloor / (float) 4.0;    // TO DO: replace by const float [Divide by num of elements]
                  // threshold = (float) 1.2 * noiseFloor;
                  stage = 1;
                  chipCount = 0;

                  // Serial.print("th: ");
                  // Serial.println(threshold);
               }
               
            break;

            case 1:
            {
               // First time entering, chipCount should be 0

               cli();
               locResult = result_f; // save a local copy of result_f in case sampChannel() triggers
               sei();

               // Do thresholding and populate the modArr 
               if (locResult > threshold) {
                  modArr[chipCount] = 1;
               } else {
                  modArr[chipCount] = 0;
               }

               chipCount++;

               if (chipCount == 3) { // TO DO: replace 4 with const int [Length of Mod Scheme] 
                  decodeModScheme();
                  stage = 2;  
                  chipCount = 0;
               }
            }

            break;

            case 2:
            {
               // First time entering, chipCount should be 0

               cli();
               locResult = result_f; // save a local copy of result_f in case sampChannel() triggers
               sei();

               // Do thresholding
               codeword = codeword << 1;

               if (locResult > threshold) {
                  codeword = codeword | 1;
                  //Serial.print("1");
               } else {
                  codeword = codeword | 0;
                  //Serial.print("0");
               }          

              //Serial.println(codeword);     

               // chipCount increments every bit, resets to zero when chipCount == DATALEN (PACKETLEN (64) - OVERHEAD (16) = DATALEN (48))
               chipCount++;
               // codewordCount also increments every bit, but resets to zero when codewordCount == codeword length of a modScheme
               codewordCount++;

               // TODO: LOOKUP TABLE FOR MOD SCHEME CODEWORDS
               // how to differentiate the different intervals to call decode data?
               decodeData();
            }

            break;

            default:
               Serial.println("ERROR!");
               cli();
               sampleCount = 0;
               chipCount = 0;
               codewordCount = 0;
               sei();
               confirmed = 0;
            break;
         }
      }

      state = TIME;

      if (chipCount == DATALEN) { // AFTER STAGE 2 COMPLETE
        /* For MATLAB: print something to indicate end of packet ---------------------------------------------------------
        Serial.print("Mod: ");
        for (int i = 0; i < 4; i++) {
            Serial.print(modArr[i]);
        }
        */
        //Serial.println(' ');
        //Serial.println(' ');
         // -- end -- */
        

         chipCount = 0;
         sampleCount = 0;
         confirmed = 0;
         stage = 0;
         // decodeData(); // THIS SHOULD NOT BE HERE IN FINAL PRODUCT
         state = TIME;
      }

      break;

   case DECODE:
      state = DECODE;

      break;

   default:
      Serial.println("ERROR!");
      cli();
      sampleCount = 0;
      chipCount = 0;
      codewordCount = 0;
      sei();
      confirmed = 0;
      state = TIME;
      break;
  }

}


static void sampChannel(void) {

  if(GPT1_SR&(GPT_SR_OF1))
  {
    GPT1_SR = GPT_SR_OF1; // write to GPT2_SR[OF1] clears the interrupt
    // Serial.println("Sampling...");

    result = analogRead(A1);
    

    sampleCount++;
    bitSampled = 1;
  }
}

static void decodeModScheme(void) {
  /* GRP 1: 8-PPM, 4-PPM,      2-PPM,      (4,3) OPPM */
  /* GRP 2: 8-PPM, (8,2) OPPM, (8,4) OPPM, (8,6) OPPM */

   // If less than 8, then no need use 3rd layer.
   if (modArr[0]) {
      if (modArr[1]) {
         if (modArr[2]) {
            // 111 - (8,6) OPPM
            modScheme = 1;
         } else {
            // 110 - (4,3) OPPM
            modScheme = 2;
         }
      } else {
         if (modArr[2]) {
            // 101 - (8,4) OPPM
            modScheme = 3;
         } else {
            // 100 - 2-PPM
            modScheme = 4;
         }
      }
   } else {
      if (modArr[1]) {
         if (modArr[2]) {
            // 011 - (8,2) OPPM
            modScheme = 5;
         } else {
            // 010 - 4-PPM
            modScheme = 6;
         }
      } else {
         if (modArr[2]) {
            // 001 - 8-PPM
            modScheme = 7;
         } else {
            // 000
            modScheme = 8;
         }
      }
   }
   
   // Serial.print("Mod:");
   // Serial.println(modScheme);
}

static void decodeData(void) {
   // use bitShift in case 2 in RECORD, then here use switch case comparing bytes to integer values

   // switch statement for mod scheme
   switch (modScheme) {
      case 1: // (8,6)
      {
        if (codewordCount == 8) {
          codewordCount = 0;
          messageCount += 3;
          message = message << 3;

          switch (codeword) {
              case 252: // codeword = b'10000000
                message = message | 0;
              break;

              case 126: // codeword = b'01000000
                message = message | 1;
              break;

              case 63: // codeword = b'00100000
                message = message | 2;
              break;

              case 159: // codeword = b'00010000
                message = message | 3;
              break;

              case 207: // codeword = b'00001000
                message = message | 4;
              break;              

              case 231: // codeword = b'00000100
                message = message | 5;
              break;

              case 243: // codeword = b'00000010
                message = message | 6;
              break;

              case 249: // codeword = b'00000001
                message = message | 7;
              break;

              default:
                //Serial.println("INV");
                message = message | 0;
              break;
          }
          codeword = 0;
        }
      }  
      break;

      case 2: // (4,3)
      {
        if (codewordCount == 4) {
          //Serial.print("code:");
          //Serial.println(codeword);
          
          codewordCount = 0;
          messageCount += 2;
          message = message << 2;

          switch (codeword) {
              case 14: // codeword = b'1110
                message = message | 0;
              break;

              case 7: // codeword = b'0111
                message = message | 1;
              break;

              case 11: // codeword = b'1011
                message = message | 2;
              break;

              case 13: // codeword = b'1101
                message = message | 3;
              break;

              default:
                //Serial.println("INV");
                message = message | 0;
              break;
          }
          codeword = 0;
        }
      }    
      break;
      
      case 3: // (8,4)
      {
        if (codewordCount == 8) {
          codewordCount = 0;
          messageCount += 3;
          message = message << 3;

          switch (codeword) {
              case 240: // codeword = b'10000000
                message = message | 0;
              break;

              case 120: // codeword = b'01000000
                message = message | 1;
              break;

              case 60: // codeword = b'00100000
                message = message | 2;
              break;

              case 30: // codeword = b'00010000
                message = message | 3;
              break;

              case 15: // codeword = b'00001000
                message = message | 4;
              break;              

              case 135: // codeword = b'00000100
                message = message | 5;
              break;

              case 195: // codeword = b'00000010
                message = message | 6;
              break;

              case 225: // codeword = b'00000001
                message = message | 7;
              break;

              default:
                //Serial.println("INV");
                message = message | 0;
              break;
          }
          codeword = 0;
        }
      }  
      break;

      case 4: // 2-PPM
      {
         if (codewordCount == 2) {
            codewordCount = 0;
            messageCount++;
            message = message << 1;

            switch (codeword) {
               case 1: // codeword = b'01
                  message = message | 1;
               break;

               case 2: // codeword = b'10
                  message = message | 0;
               break;

               default:
                  //Serial.println("INV");
                  message = message | 0;
               break;
            }
            codeword = 0;
         }
      }
      break;

      case 5: // (8,2)
      {
        if (codewordCount == 8) {
          codewordCount = 0;
          messageCount += 3;
          message = message << 3;

          switch (codeword) {
              case 192: // codeword = b'10000000
                message = message | 0;
              break;

              case 96: // codeword = b'01000000
                message = message | 1;
              break;

              case 48: // codeword = b'00100000
                message = message | 2;
              break;

              case 24: // codeword = b'00010000
                message = message | 3;
              break;

              case 12: // codeword = b'00001000
                message = message | 4;
              break;              

              case 6: // codeword = b'00000100
                message = message | 5;
              break;

              case 3: // codeword = b'00000010
                message = message | 6;
              break;

              case 129: // codeword = b'00000001
                message = message | 7;
              break;

              default:
                //Serial.println("INV");
                message = message | 0;
              break;
          }
          codeword = 0;
        }
      }  
      break;

      case 6: // 4-PPM
      {
        if (codewordCount == 4) {
          codewordCount = 0;
          messageCount += 2;
          message = message << 2;

          switch (codeword) {
              case 8: // codeword = b'1110
                message = message | 0;
              break;

              case 4: // codeword = b'0111
                message = message | 1;
              break;

              case 2: // codeword = b'1011
                message = message | 2;
              break;

              case 1: // codeword = b'1101
                message = message | 3;
              break;

              default:
                //Serial.println("INV");
                message = message | 0;
              break;
          }
          codeword = 0;
        }
      }  
      break;

      case 7: // 8-PPM
      {
        if (codewordCount == 8) {
          
          // Serial.print("code:");
          // Serial.println(codeword);

          codewordCount = 0;
          messageCount += 3;
          message = message << 3;

          // Serial.print("msgCount:");
          // Serial.println(messageCount);

          switch (codeword) {
              case 128: // codeword = b'10000000
                message = message | 0;
              break;

              case 64: // codeword = b'01000000
                message = message | 1;
              break;

              case 32: // codeword = b'00100000
                message = message | 2;
              break;

              case 16: // codeword = b'00010000
                message = message | 3;
              break;

              case 8: // codeword = b'00001000
                message = message | 4;
              break;              

              case 4: // codeword = b'00000100
                message = message | 5;
              break;

              case 2: // codeword = b'00000010
                message = message | 6;
              break;

              case 1: // codeword = b'00000001
                message = message | 7;
              break;

              default:
                //Serial.println("INV");
                message = message | 0;
              break;
          }
          codeword = 0;
        }
      }  
      break;

      case 8: // (4,2) OPPM
      {
        if (codewordCount == 4) {
          codewordCount = 0;
          messageCount += 2;
          message = message << 2;

          switch (codeword) {
              case 12: // codeword = b'1110
                message = message | 0;
              break;

              case 6: // codeword = b'0111
                message = message | 1;
              break;

              case 3: // codeword = b'1011
                message = message | 2;
              break;

              case 9: // codeword = b'1101
                message = message | 3;
              break;

              default:
                //Serial.println("INV");
                message = message | 0;
              break;
          }
          codeword = 0;
        }
      }  
      break;

      default:
         Serial.println("ERROR");
      break;
   }

  if (messageCount >= 8) {
    messageCount -= 8;
    message8bit = message >> messageCount; // take the 8 MSB
    
    // FOR HELLO WORLD
    //byte temp = message8bit;

    // FOR IMAGE
    uint8_t temp = message8bit;

    // byte temp = message; // truncate the 8 LSB (TEST WHETHER THIS WORKS)
    //Serial.print((char) temp);
    Serial.print(temp);
    Serial.print(',');

    switch (messageCount) {
      case 0:
        message = message & 0;
      break;
      case 1:
        message = message & 1;
      break;
      case 2:
        message = message & 3;
      break;
      default:
        message = message & 0;
        Serial.println("Invalid msg!");
      break;
    }
    
    //Serial.print("shift:");
    //Serial.println(message);
  }
   
   
   // Print stats
   /*
   Serial.print("Final Data: ");
   Serial.println(dataStr);
   Serial.print("Noise Floor: ");
   Serial.println(noiseFloor);
   Serial.print("Threshold: ");
   Serial.println(threshold);
   */

}

static void reset(void) {
  // Reset Flags
  bitSampled = 0;
  detected = 0;
  confirmed = 0;
  // Reset Counters
  sampleCount = 0;
  chipCount = 0;
  codewordCount = 0;
  messageCount = 0;
  // Reset Arrays
  detection = 0;
  searchBitArr = 0;
  //memset(boolSearchArr, 0, sizeof(boolSearchArr));
  memset(searchArr, 0, sizeof(searchArr));
  // Reset Arrows
  // s1 = 0;
  // s2 = 24;
  // s3 = 20;
  // s4 = 16;
  // s5 = 12;
  // s6 = 8;
  // s7 = 4;
  // Reset Output
  codeword = 0;
  message = 0;
}