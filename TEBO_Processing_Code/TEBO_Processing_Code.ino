#include <ILI9341_T4.h> 
#include <tgx.h> 
#include <Bounce2.h>
#include <Encoder.h>

#include <ADC.h>
#include <IntervalTimer.h>

#include <font_tgx_OpenSans_Bold.h>
#include "font_tgx_OpenSans.h"
#include "font_tgx_Arial_Bold.h"

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define GREEN   0x07E0
#define WHITE   0xFFFF
#define BLUE    0x001F
#define RED     0xF800
#define YELLOW  0xFFE0
#define CYAN    0x07FF

using namespace tgx;

#define PIN_SCK     13      // mandatory
#define PIN_MISO    12      // mandatory
#define PIN_MOSI    11      // mandatory
#define PIN_DC      23     

#define PIN_CS      255      // optional (but recommended), can be any pin.  
#define PIN_RESET   25       // optional (but recommended), can be any pin. 
#define PIN_BACKLIGHT 24   // optional, set this only if the screen LED pin is connected directly to the Teensy.
#define PIN_TOUCH_IRQ 255   // optional. set this only if the touchscreen is connected on the same SPI bus
#define PIN_TOUCH_CS  255   // optional. set this only if the touchscreen is connected on the same spi bus

#define SPI_SPEED       40000000

// the screen driver object
ILI9341_T4::ILI9341Driver tft(PIN_CS, PIN_DC, PIN_SCK, PIN_MOSI, PIN_MISO, PIN_RESET, PIN_TOUCH_CS, PIN_TOUCH_IRQ);

// 2 x 10K diff buffers (used by tft) for differential updates (in DMAMEM)
DMAMEM ILI9341_T4::DiffBuffStatic<6000> diff1;
DMAMEM ILI9341_T4::DiffBuffStatic<6000> diff2;

// screen dimension (landscape mode)
static const int SLX = 320;
static const int SLY = 240;

// main screen framebuffer (150K in DTCM for fastest access)
DMAMEM uint16_t fb[SLX * SLY];

// internal framebuffer (150K in DMAMEM) used by the ILI9431_T4 library for double buffering.
DMAMEM uint16_t internal_fb[SLX * SLY];

// image that encapsulates fb.
Image<RGB565> im(fb, SLX, SLY);

ADC *adc = new ADC();
bool paused = false;
bool enable_math = false;

bool draw_YCH1 = false;
bool draw_YCH2 = false;
bool adjust_trigger = false;

int cursor1_xpos = 160;
int cursor1_ypos = 120;
int cursor2_xpos = 160;
int cursor2_ypos = 120;

#define NUM_SAMPLES 640
int sample_iterator;
int channel1_raw[NUM_SAMPLES];
int channel2_raw[NUM_SAMPLES];
int ch1_pxl[NUM_SAMPLES];
int ch2_pxl[NUM_SAMPLES];

#define CHANNEL_A   14
#define CHANNEL_B   19
#define ADC_RESOLUTION    10      // Resolution in bits
#define ADC_OVERSAMPLING  16      // 
#define SAMPLING_INTERVAL 1      // microseconds
volatile float time_scale = 1.0;
long preserved_time_encoder = 0;

IntervalTimer sampling_timer;

#define ROT_1A 2  // LEFT Encode Clockwise
#define ROT_1B 3  // LEFT Encode Counter-Clockwise
#define ROT_1C 4  // LEFT Encode Button
#define ROT_2A 5  // RIGHT Encode Clockwise
#define ROT_2B 6  // RIGHT Encode Counter-Clockwise
#define ROT_2C 7  // RIGHT Encode Button
#define PAUSE_PIN 9 // Bottom Button
#define MATH_PIN 22 // Top Button

Bounce pauseButton = Bounce();
Bounce mathButton = Bounce();
Bounce encoderAButton = Bounce();
Bounce encoderBButton = Bounce();

#define TRIGGER_MIN 0
#define TRIGGER_MAX 1023
#define TRIGGER_STEP 10  // Step size per encoder tick
#define TRIGGER_TIMEOUT 1000 
#define TRIGGER_CHANNEL channel1_raw

Encoder encoderA(ROT_1A, ROT_1B);
Encoder encoderB(ROT_2A, ROT_2B);

#define SCALE_MIN 0.1
#define SCALE_MAX 5.0
#define SCALE_STEP 0.1
volatile float vertical_scale = 1.0;
long preserved_voltage_encoder = 0;

bool triggered = false;
volatile int trigger_level = 512;
volatile bool show_trigger = false;
unsigned long last_encoder_activity = 0;
long preserved_trigger_encoder = 0;

void update_trigger() {
  long new_position = encoderB.read() / 4;  // Adjust sensitivity

  if (new_position != preserved_trigger_encoder) {
    preserved_trigger_encoder = new_position;
    trigger_level = constrain(512 + (new_position * TRIGGER_STEP), TRIGGER_MIN, TRIGGER_MAX);
    show_trigger = true;
    last_encoder_activity = millis();  // Reset timer
  }

  // Hide trigger if inactive for more than TRIGGER_TIMEOUT
  if (millis() - last_encoder_activity > TRIGGER_TIMEOUT) {
    show_trigger = false;
  }
}

void update_voltage_scaling() {
  long new_position = encoderB.read() / 4;

  if (new_position != preserved_voltage_encoder) {
    preserved_voltage_encoder = new_position;
    vertical_scale = constrain(1.0 + (new_position * SCALE_STEP), SCALE_MIN, SCALE_MAX);
    Serial.printf("Vertical Scale: %.1fx\n", vertical_scale);
  }
}

// NOT SURE IF THIS IS CORRECT AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
void update_time_scaling() {
  long new_position = encoderA.read() / 4;  // Adjust sensitivity

  if (new_position != preserved_time_encoder) {
    preserved_time_encoder = new_position;
    time_scale = constrain(time_scale + (new_position * 0.1), 0.2, 10.0); // Adjust the range as needed
    Serial.printf("Time Scale: %.1f\n", time_scale);
  }
}


void sample(){

  while(adc->adc0->isConverting() || adc->adc1->isConverting());

  int ch1_value = 1023 - adc->adc0->readSingle();
  int ch2_value = 1023 - adc->adc1->readSingle();

  channel1_raw[sample_iterator] = ch1_value;
  channel2_raw[sample_iterator] = ch2_value;

  ch1_pxl[sample_iterator] = map(channel1_raw[sample_iterator],0,1023,0,239);
  ch2_pxl[sample_iterator] = map(channel2_raw[sample_iterator],0,1023,0,239);

  adc->startSynchronizedSingleRead(CHANNEL_A, CHANNEL_B);

  if(sample_iterator == (NUM_SAMPLES - 1)){
    sampling_timer.end();
    sample_iterator = 0;
  }else{
    sample_iterator++;
  }
  
}

void draw_math_functions(){
  if(!enable_math){
    return;
  }
}

void draw_channel_data(){
  
  if(!enable_math && adjust_trigger){
    update_trigger();
  }
  int trigger_pixel = map(trigger_level, 0, 1023, 239, 0);
  if(!enable_math && !adjust_trigger){
    update_voltage_scaling();
  }
  if(!enable_math){
    update_time_scaling();
  }

  /*for (int i = 0; i < (320 * time_scale); i += time_scale) {
    for(int j = 0; j < 240; j++){
      int ch1_scaled = constrain((ch1_pxl[i] - 120) * vertical_scale + 120, 0, 239);
      int ch2_scaled = constrain((ch2_pxl[i] - 120) * vertical_scale + 120, 0, 239);

      if(ch1_scaled == j){
        im((i / time_scale), ch1_scaled) = tgx::RGB565_Blue;
      }else if(ch2_scaled == j){
        im((i / time_scale), ch2_scaled) = tgx::RGB565_Yellow;
      }else{
        im((i / time_scale), j) = tgx::RGB565_Black;  
      }
    }
  }*/

  im.clear(tgx::RGB565_Black);
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int ch1_scaled = constrain((ch1_pxl[i] - 120) * vertical_scale + 120, 0, 239);
    int ch2_scaled = constrain((ch2_pxl[i] - 120) * vertical_scale + 120, 0, 239);
    
    int x = i * time_scale;
    if (x >= SLX) break; // Stop drawing if x exceeds screen width
    im(x, ch1_scaled) = tgx::RGB565_Blue; 
    im(x, ch2_scaled) = tgx::RGB565_Yellow;
  }

  if (show_trigger) {
    for (int x = 0; x < SLX; x++) {
      im(x, trigger_pixel) = tgx::RGB565_Red;
    }
    Serial.printf("\nTrigger Level: %d %s", trigger_level, show_trigger ? "[VISIBLE]" : "[HIDDEN]");
  }
}

void setup(void) {
  Serial.begin(115200);
  sample_iterator = 0;

  encoderA.write(0);
  encoderB.write(0);

  pinMode(ROT_1A, INPUT_PULLUP);
  pinMode(ROT_1B, INPUT_PULLUP);
  pinMode(ROT_1C, INPUT_PULLUP);
  pinMode(ROT_2A, INPUT_PULLUP);
  pinMode(ROT_2B, INPUT_PULLUP);
  pinMode(ROT_2C, INPUT_PULLUP);

  pinMode(PAUSE_PIN, INPUT_PULLUP);
  pinMode(MATH_PIN, INPUT_PULLUP);

  pauseButton.attach(PAUSE_PIN, INPUT_PULLUP);
  pauseButton.interval(5); // 5 ms debounce interval
  mathButton.attach(MATH_PIN, INPUT_PULLUP);
  mathButton.interval(5); // 5 ms debounce interval
  encoderAButton.attach(ROT_1C, INPUT_PULLUP);
  encoderAButton.interval(5); // 5 ms debounce interval
  encoderBButton.attach(ROT_2C, INPUT_PULLUP);
  encoderBButton.interval(5); // 5 ms debounce interval

  adc->adc0->setResolution(ADC_RESOLUTION);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setAveraging(ADC_OVERSAMPLING);
  adc->adc1->setResolution(ADC_RESOLUTION);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc1->setAveraging(ADC_OVERSAMPLING);

  tft.output(&Serial);
  // initialize the ILI9341 screen
  while(!tft.begin(SPI_SPEED));

  // ok. turn on backlight
  pinMode(PIN_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);

  tft.setRotation(1); // portrait mode
  tft.setFramebuffer(internal_fb); // double buffering
  tft.setDiffBuffers(&diff1, &diff2); // 2 diff buffers
  tft.setDiffGap(4); // small gap
  tft.setRefreshRate(140); // refresh at 60hz
  tft.setVSyncSpacing(2);
  im.clear(tgx::RGB565_Black);

  adc->startSynchronizedSingleRead(CHANNEL_A, CHANNEL_B); // start ADC, read A0 and A1 channels
  sampling_timer.begin(sample, SAMPLING_INTERVAL);
  Serial.println("SETUP DONE");
}
void loop(void) {

  pauseButton.update();
  mathButton.update();
  encoderAButton.update();
  encoderBButton.update();
  
  if(encoderBButton.fell()){
    adjust_trigger = !adjust_trigger;
    if(adjust_trigger){
      encoderB.write(preserved_trigger_encoder * 4);
    }else{
      encoderB.write(preserved_voltage_encoder * 4);
    }
    Serial.printf("\nTRIGGER Status: %d", adjust_trigger);
  }
  if(pauseButton.fell()) {
    paused = !paused; // Toggle pause state
    sampling_timer.end();
    Serial.printf("\nPAUSE Status: %d", paused);
  }
  if(mathButton.fell()){
    enable_math = !enable_math;
    if(adjust_trigger){
      encoderB.write(preserved_trigger_encoder * 4);
    }else{
      encoderB.write(preserved_voltage_encoder * 4);
    }
    encoderA.write(preserved_time_encoder * 4);
    Serial.printf("\nMATH Status: %d", enable_math);
  }


  if(!paused){
    draw_channel_data();
    draw_math_functions();
    tft.update(fb);
    sampling_timer.begin(sample, SAMPLING_INTERVAL);
  }
}


