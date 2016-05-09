//FULL TOWER OF WINDS CODE
//includes: peak detection, peak results in short light pulse
//variance in (phi, theta) space of random walk proportional to spectral data:
//bass increases variance in phi, trebel in theta
//that one'll kinda be hard to see but w/e
//FFT code modified from Adafruit Piccolo Project, https://learn.adafruit.com/piccolo/code

#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>

#define ADC_CHANNEL 0

int pinNums[] = {5, 6, 3}; //location of red, green, blue pins
const float pi = 3.14;
//setting r higher reduces flicker
int r_default = 300;
int r_peak = 0; //turns off 
int currentSpherical[] = {300, 45, 45}; //r, phi, theta, in degrees because integer operations are easier
int currentRGB[3]; 
int targetRGB[3]; 
int targetSpherical[3];


//AUDIO PROCESSING STUFF
int low; //for generating new points
int hi;
uint16_t val;
int avg = 500; //moving average, initialized to 500
int diff;
const int samples = 20; //moving average length
const float peak = 1.20; //peak if reading is this multiple of mean
bool fadeBack = false; //fading back up
bool sampling_fft = true; 
//filtering and processing values
 static const uint8_t PROGMEM
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
   // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },

  //I changed this from the piccolo code a bit, so the data storage makes more sense
  //column-wise number of bins to merge
  numBins[8] = {2, 4, 5, 8, 11, 17, 25, 37},
  //index of first bin for this final bin
  firstIndex[8] = {1, 1, 2, 3, 5, 7, 11, 16}, //looks like a lot of overlap here
  //weights for each bin
  col0data[] = {111, 8 },        
  col1data[] = {19, 186,  38,   2 },
  col2data[] = {11, 156, 118,  16,   1 },
  col3data[] = {5,  55, 165, 164,  71,  18,   4,   1 },
  col4data[] = {3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 },
  col5data[] = {  2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 },
  col6data[] = { 1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  col7data[] = {1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 },
  // And then this points to the start of the data for each of the columns:
  * const colData[]  = {
    col0data, col1data, col2data, col3data,
    col4data, col5data, col6data, col7data };
 
int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer, this is half the size of the FFT?
volatile byte samplePos = 0;     // Buffer position counter
int            processed_vals[8]; //values
int            final_vals[4]; //values that will be used to weight directions
const int      direction_default[4] = {15, 15, 15, 15};
int            step_variance[4];
int            colDiv[8]; //divisors for each column, for weighted average


LiquidCrystal lcd(7, 8, 9, 10, 11, 12); //start LCD

//enables free-running mode
void start_freerun(){
  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin
  TIMSK0 = 0;                // Timer0 off

  sei(); // Enable interrupts
}

float degToRad(int deg){
  return ((float)deg)*pi/180;
}

//sets the LEDs to a specific color in RGBspace
void setColorRGB(int *color){
  for (int i = 0; i < 3; i++){
    analogWrite(pinNums[i], color[i]);
  }
}

//convert from float to RGB cartesian color, places it in rgb
void sph_to_rgb(int *sph, int *rgb){
  rgb[0] = (int)(sph[0]*sin(degToRad(sph[1]))*cos(degToRad(sph[2])));
  rgb[1] = (int)(sph[0]*sin(degToRad(sph[1]))*sin(degToRad(sph[2])));
  rgb[2] = (int)(sph[0]*cos(degToRad(sph[1])));

  //constrained to RGB color space
  for (int i = 0; i < 3; i++){
    rgb[i] = constrain(rgb[i], 0, 255);
  }
}

void LEDinit(int *pins){
  for (int i = 0; i < 3; i++){
    pinMode(pins[i], OUTPUT);
  }
}

//generate next walk step on the sphere
//vars is a length-4 array with delta_phi and delta_theta in both directions
void nextTarget(int *current, int *target, int *deltas){
  target[0] = current[0]; //keep r constant during a step  
  low = max(0, current[1] - deltas[0]);
  hi = min(90, current[1] + deltas[1]);   
  target[1] = random(low, hi);
  low = max(0, current[2] - deltas[2]);
  hi = min(90, current[2] + deltas[3]); 
  target[2] = random(low, hi);
  //Serial.println(String(target[1]) + " " + String(target[2]));
}

//using regularized step like  https://www.arduino.cc/en/Tutorial/ColorCrossfader
//except modified for stepping through angles
//will probably have some issue due to rounding?
int calculateStep(int prevValue, int endValue) {
  int stepSize = endValue - prevValue; // What's the overall gap?
  if (stepSize) {                      // If its non-zero, 
    stepSize = 3600/stepSize;              //   divide by 360, 4*90
  } 
  return stepSize;
}

void updateAngles(int *currentSph, int step_phi, int step_theta, int stepnum){
  //updating phi
  if ((step_phi) && stepnum % step_phi == 0){ //incrementing or decrementing phi if it's time to step
    if (step_phi > 0){
      currentSph[1] += 1;
    } else if (step_phi < 0){
      currentSph[1] -= 1;
    }
    currentSph[1] = constrain(currentSph[1], 0, 90); //juuuust in case
  }
   if ((step_theta) && stepnum % step_theta == 0){ //incrementing or decrementing phi if it's time to step
    if (step_theta > 0){
      currentSph[2] += 1;
    } else if (step_theta < 0){
      currentSph[2] -= 1;
    }
    currentSph[2] = constrain(currentSph[2], 0, 90); //juuuust in case
  }
}

void crossFadeSph(int *currentSph, int *targetSph, int *currentRGB, int timestep){
  int step_phi = calculateStep(currentSph[1], targetSph[1]);
  int step_theta = calculateStep(currentSph[1], targetSph[1]);

  for (int i = 0; i <=3600; i++){
    //checking here for a peak event - this allows slow transitions between colors while still detecting things
    while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish, will be in val
    diff = (int)val - avg;
    avg += diff/samples;

    //Serial.println(avg);

    if (val > avg*peak && !fadeBack){ //ignore if fading back already started
      currentSph[0] = r_peak;
      //Serial.println("Peak Detected! Value: " + String( val) + " Average: " + String(avg));
      updateLCDPeak();
      fadeBack = true;
    }

    if (fadeBack){
      if (currentSph[0] < r_default){
        currentSph[0] += 3; 
      } else{
        currentSph[0] = r_default; //correct for overshoot
        fadeBack = false;
        //Serial.println("back to starting brightness");
      }
    }
    
    updateAngles(currentSph, step_phi, step_theta, i);
    sph_to_rgb(currentSph, currentRGB);
    setColorRGB(currentRGB);
    ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
    delayMicroseconds(timestep);
  }
  //back to default at end of cycle, probably some kind of fade
  currentSph[0] = r_default;
  fadeBack = false;
}

String printVec(int *vec, int len){
  String str = "";
  for (int i = 0; i < len; i++){
    str += String(vec[i]) + " ";
  }

  str += "         ";
  
  return str;
}

void updateLCDDeltas(int *deltas){
  lcd.clear();
  lcd.print("New Deltas (deg)");
  lcd.setCursor(0, 1);
  lcd.print(printVec(deltas, 4));
}

void updateLCDPeak(){
  lcd.clear();
  lcd.print("Peak detected!");
  lcd.setCursor(0, 1);
  lcd.print("p: " + String(val) + " a: " + String(avg));
}

void updateLCDColors(int *rgb, int *sph){
  lcd.clear();
  lcd.print("sph:");
  lcd.setCursor(0, 1);
  lcd.print("rgb:");
  lcd.setCursor(4, 0);
  lcd.print(printVec(sph, 3));
  lcd.setCursor(4, 1);
  lcd.print(printVec(rgb, 3));
}

void setup() {
  randomSeed(analogRead(A1)); //read from pin
  LEDinit(pinNums);
  //Serial.begin(9600);
  lcd.begin(16, 2);
  sph_to_rgb(currentSpherical, currentRGB); //initializes color
  setColorRGB(currentRGB);
  updateLCDColors(currentSpherical, currentRGB);

  uint8_t i, j, nBins, binNum, *data;
   //initialize divisor for weighted average

  for(i=0; i<8; i++) {
    data   = (uint8_t *)pgm_read_word(&colData[i]);
    nBins  = pgm_read_byte(&numBins[i]);
    binNum = pgm_read_byte(&firstIndex[i]);
    for(colDiv[i]=0, j=0; j<nBins; j++)
      colDiv[i] += pgm_read_byte(&data[j]);
  }

  //begin free-running
  start_freerun();
}

//gets an FFT sample, uses it to update the variances in the phi and theta directions
void update_variances(){
  uint8_t  i, x, L, *data, nBins, binNum, weighting, c;
  int    sum;
  sampling_fft = true; //sampling from FFT now

  while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish

  //performs FFT on the sample
  fft_input(capture, bfly_buff);   // Samples -> complex #s
  samplePos = 0;                   // Reset sample counter
  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
  fft_execute(bfly_buff);          // Process complex data
  fft_output(bfly_buff, spectrum); // Complex -> spectrum

  // Remove noise and apply EQ levels
  for(x=0; x<FFT_N/2; x++) {
    L = pgm_read_byte(&noise[x]);
    //if the spectrum is less than equal to the noise level, set it to 0
    spectrum[x] = (spectrum[x] <= L) ? 0 :
      (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
  }

    // Downsample spectrum output to 8 columns:
  for(x=0; x<8; x++) {
    data   = (uint8_t *)pgm_read_word(&colData[x]);
    nBins  = pgm_read_byte(&numBins[x]);
    binNum = pgm_read_byte(&firstIndex[x]);
    for(sum=0, i=0; i<nBins; i++){
      sum += spectrum[binNum++] * pgm_read_byte(&data[i]); 
    }
    processed_vals[x]= sum / colDiv[x];                    // weighted average
  }

  //processing column values into final values
  for (int i = 0; i < 4; i++){
    final_vals[i] = abs(processed_vals[2*i] + processed_vals[2*i + 1]); //takes absolute value
  }
  //Serial.println("Processed Column Values: " + col_vals(processed_vals, 8));
  //Serial.println("Binned Values: " + col_vals(final_vals, 4));

  //modify the random walk step range
  // (phi_low, phi_hi, theta_low, theta_high)
  //large values increase the step range, small values decrease it
  //minimum step range = 15 degrees, 
  //sum the value to the step range, with a max of 30 degrees (out-of-range handled by random walking)

  for (int i = 0; i < 4; i++){
    step_variance[i] = 3*final_vals[i] + direction_default[i]; //multiply by 3 to scale
    step_variance[i] = constrain(step_variance[i], 15, 30);
  }
}

void loop() {
  //generates a new target
  update_variances();
  updateLCDDeltas(step_variance);
  sampling_fft = false;
  nextTarget(currentSpherical, targetSpherical, step_variance);
  sph_to_rgb(targetSpherical, targetRGB);
 // Serial.println("RGB: " + printVec(targetRGB) + " Sph:" + printVec(targetSpherical));
  crossFadeSph(currentSpherical, targetSpherical, currentRGB, 10);
  updateLCDColors(targetRGB, targetSpherical);
  //delay(50);
}

//this interrupt is called whenever the conversion is done, I think
ISR(ADC_vect) { // Audio-sampling interrupt
  static const int16_t noiseThreshold = 4;
  int16_t              sample         = ADC; // checks the ADC register? 
  bool done = false; //now two conditions that trigger completion - one sample if we're shifting LEDs, FFT_N if we're getting an FFT sample

  if (sampling_fft){
    capture[samplePos] =
    ((sample > (512-noiseThreshold)) &&
     (sample < (512+noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511

    if(++samplePos >= FFT_N) done = true; //gathered all of the FFT data we need, quit sampling
  } else{ //we're in the cycle loop, just get one sample
    val = sample; 
    done = true; //only needed one! we're done
  }

  if (done){
    ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off, report back to main loop
  }
}
