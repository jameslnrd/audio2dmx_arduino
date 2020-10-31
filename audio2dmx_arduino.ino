
#include <Conceptinetics.h>
#include "IRremote.h"
#include <EEPROM.h>


//////////////////////////// DMX SHIELD STUFF

// CTC-DRA-13-1 ISOLATED DMX-RDM SHIELD JUMPER INSTRUCTIONS
#define DMX_MASTER_CHANNELS   20 

// Pin number to change read or write mode on the shield
#define RXEN_PIN                2

// Configure a DMX master controller, the master controller
// will use the RXEN_PIN to control its write operation 
// on the bus
DMX_Master        dmx_master ( DMX_MASTER_CHANNELS, RXEN_PIN );


///////// IR RECEIVER STUFF 
int receiver5v = 5;  // Signal Pin to create a logic 5V to power the IR detector
int receiverPin = 6; // Signal Pin of IR receiver to Arduino Digital Pin 6
int receiverGnd = 7; // Signal Pin to create ground for IR module

IRrecv irrecv(receiverPin);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'


// SAMPLE VARIANCE CIRCULAR BUFFER
# define BUF_SIZE 50
float signal_buffer[BUF_SIZE];
int buf_index = 0;


//////////// PIN AND SENSOR STUFF
int soundPin = A0;
int sensorValue = 0;

//int ledPow = LED_BUILTIN;
//int ledGnd = 6;

float sampleVariance = 0;
float sensitivity = 1;

// SMOOTHED SLIDING ATTRIBUTES
float y[2];
int slideUpFactor = 3;
int slideDownFactor = 20;
int curSlide = 0;  

float yAvg[2];
int slideUpFactor2 = 50;
int slideDownFactor2 = 50;


// "HIGH PASSED ABSOLUTE VARIATION" CALCULATION AND SMOOTHING
float EMA_a = 0.1;    //initialization of EMA alpha
int EMA_S = 0;        //initialization of EMA S
int highpass = 0;

float absHigh = 0;
float prevAbsHigh = 0;
float sig = 0;

float feature1, feature2;
int varVal[2] = {0, 0};
int hpVal[2] = {0, 0};
int combinedVal[2];

// For RGB operating modes
byte colors[3][3] = {{1,2,3},{3,2,1}, {2,1,3}};
byte curColor = 0;

// For monochromatic modes
byte monoMode = 0;
byte monoCol = 0;

// EEPROM STUFF
/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int sensAddr = 0;
int colAddr = 1;
int monoAddr = 2;
int monoColAddr = 3;

///// UTILITY FUNCTIONS

/* Calculate the sample variance on a buffer of BUF_SIZE data points */
float sample_variance(float *signal, unsigned int signal_length, int curBufIndex)
{
  float sum = 0.0;
  float sum2 = 0.0;
  int index = 0;
  for (unsigned int i=0; i<signal_length; i++)
  {
    index = (i+curBufIndex)%signal_length;
    sum += signal[index];
    sum2 += signal[index]*signal[index];
  }

  float N = (float)signal_length;
  return (sum2 - sum*sum/N)/(N-1.0);
}

/* Signal smoothing algorithm: take sample, mem structure and slide up/down params */
float smooth_signal(float sample, float* mem, int slideUp, int slideDown){

  // Calculate current slide value
  int curSlide = slideUp;
  if(sample < mem[1])
    curSlide = slideDown;

  // update memory structure
  mem[0] = mem[1] + (sampleVariance - mem[1])/ curSlide;
  mem[1] = mem[0];

  return mem[0];
}


void dmxBlinker(int nb){
  for(unsigned int i = 0; i < nb; i++){
    dmx_master.setChannelValue (1, 5);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(90);                       // wait for a second
    dmx_master.setChannelValue (1, 0);
    digitalWrite(LED_BUILTIN, LOW);
    delay(90);
  }  
}


int senSclale(){
  if(sensitivity < 6)
    return 1;
  else if (sensitivity < 25)
    return 3;
  return 6;
}

void increaseSensitivity(){
  sensitivity = constrain(sensitivity + senSclale(), 0, 100); 
}

void decreaseSensitivity(){
  sensitivity = constrain(sensitivity - senSclale(), 0, 100); 
}


void resetDmxChans(int chans){
  for(int i = 1; i <= chans; i++)
    dmx_master.setChannelValue (i, 0);

}

/********** Interpret IR Remote Commands *********/
void translateIR() // takes action based on IR code received
{
  switch(results.value)
  {
      // Switches between RGB and monochromatic operation modes
      case 0xFF6897: // Mode zero: RGB operation 
        monoMode = 0;
        resetDmxChans(3);
        dmxBlinker(4);

        break;
      case 0xFF30CF: // Mode 1: monochromatic red
        monoMode = 1;
        monoCol = 1; // DMX channel
        resetDmxChans(3);
        dmxBlinker(5);
        break;
      case 0xFF18E7: // Mode 2: monochromatic green
        monoMode = 1;
        monoCol = 2; // DMX channel
        resetDmxChans(3);
        dmxBlinker(6);
        break;
      case 0xFF7A85: // Mode 3: monochromatic blue
        monoMode = 1;
        monoCol = 3; // DMX channel
        resetDmxChans(3);
        dmxBlinker(7);
        break;
 
      case 0xFFE21D: // Func/Stop button : write to EEPROM Memory
        EEPROM.update(sensAddr, sensitivity);
        EEPROM.update(colAddr, curColor);
        EEPROM.update(monoAddr, monoMode);
        EEPROM.update(monoColAddr, monoCol);
        
        // Make the LED blink so we know the update worked
        dmxBlinker(10);
        break;

      case 0xFFC23D: // Fast forward button: change colour profile
        curColor = (curColor+1)%3; 
        dmxBlinker(1);
        break;

      case 0xFFA857: // Volume Down Button
        decreaseSensitivity();
        dmxBlinker(2);
        break;
        
      case 0xFF629D: // Volune Up Button
        increaseSensitivity();
        dmxBlinker(3);
        break;
      
      default: break; 
  }
}


void setup () 
{
  // Start the Infra Red receiver
  irrecv.enableIRIn();

  // create a 5V pin on 5 and 0 on pin 7 so we can connect the IR Receiver
  pinMode(receiver5v, OUTPUT);
  digitalWrite(receiver5v, HIGH);
  pinMode(receiverGnd, OUTPUT);
  digitalWrite(receiverGnd, LOW);

  // Signal Pin from the IR receiver
  pinMode(receiverPin, INPUT);


  // place a led between digital pins 7 and 6 (used to acknowledge IR commands)
  pinMode(LED_BUILTIN, OUTPUT);

  //pinMode(ledPow, OUTPUT);
  //pinMode(ledGnd, OUTPUT);  

  digitalWrite(LED_BUILTIN, LOW);

  
  // read a byte from the current address of the EEPROM
  sensitivity = (int)EEPROM.read(sensAddr);
  curColor = EEPROM.read(colAddr);

  monoMode = EEPROM.read(monoAddr);
  monoCol = EEPROM.read(monoColAddr);

  
  //set EMA S for t=1
  EMA_S = analogRead(soundPin);

  // Initialise the signal buffer to currently read voltage
  for(unsigned int i = 0; i < BUF_SIZE; i++)
    signal_buffer[i] = EMA_S; 

  // Initialise smoothing functions
  y[0] = y[1] = 0;
  yAvg[0] = yAvg[1] = 0;
  
  // Enable DMX master interface and start transmitting
  dmx_master.enable ();  
  
  // Set channel 1 - 50 @ 50%
  dmx_master.setChannelRange ( 1, 25, 0 );
}

 
void loop () 
{

  // have we received an IR signal?
  if (irrecv.decode(&results)) {
    translateIR(); 
    irrecv.resume();
  }  

  // Read the raw sensor value from the sound module
  sensorValue = analogRead (soundPin);

  // Put it into a circular buffer
  signal_buffer[buf_index] = (float)sensorValue;

  // Calculate and filter the sample variance data
  sampleVariance = sample_variance(signal_buffer, BUF_SIZE, buf_index);

  // Smooth out the first feature based on sample variance
  feature1 = smooth_signal(sampleVariance, y, slideUpFactor, slideDownFactor);

  // Calculate and filter the highpass mean stuff
  EMA_S = (EMA_a*sensorValue) + ((1-EMA_a)*EMA_S);
  highpass = sensorValue - EMA_S;         
  
  prevAbsHigh = absHigh;
  absHigh = highpass;

  // Smooth out second feature based on delta of absolute high-passed signal
  feature2 = smooth_signal(abs(absHigh-prevAbsHigh), yAvg, slideUpFactor2, slideDownFactor2);

  // Condition the data for DMX send
  varVal[0] = constrain(0.04*feature1*sensitivity, 0., 255.);
  hpVal[0] = constrain(0.1*feature2*sensitivity, 0., 255.);
  //if(hpVal[0] < 1)
  //  hpVal[0] = 0;

  // RGB operating mode
  if(monoMode == 0){
    // Only actually send the DMX commands if there is a value change !
    if(varVal[0] != varVal[1]){
      dmx_master.setChannelValue ( colors[curColor][1], varVal[0]);
      varVal[1] = varVal[0];
    }
    if(hpVal[0] != hpVal[1]){
      dmx_master.setChannelValue ( colors[curColor][2], hpVal[0]);
      hpVal[1] = hpVal[0];  
    }
  
    // Set the third channel leds if we are saturating the other channel
    if(hpVal[0] > 254)
      dmx_master.setChannelValue (colors[curColor][0],150);
    else
      dmx_master.setChannelValue (colors[curColor][0], 0);
  } 
  
  // Monochromatic operating mode
  else {
    combinedVal[0] = (hpVal[0]+varVal[0]) /2;
    if(combinedVal[0] != combinedVal[1]){
      dmx_master.setChannelValue ( monoCol, combinedVal[0]);
      combinedVal[1] = combinedVal[0];    
    }
  }
  
  // Increment the circular buffer index
  buf_index = (buf_index+1)%BUF_SIZE;

  // 1ms delay before next exectution of loop
  delay (1);
}
