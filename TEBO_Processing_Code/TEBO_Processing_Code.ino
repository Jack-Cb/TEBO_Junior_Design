#include <ILI9341_T4.h> 
#include <tgx.h> 
#include <Bounce2.h>

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

// let's not burden ourselves with the tgx:: prefix
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

#define NUM_SAMPLES 320
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

void sample(){

  while(adc->adc0->isConverting() || adc->adc1->isConverting());
  channel1_raw[sample_iterator] = 1023 - (adc->adc0->readSingle());
  channel2_raw[sample_iterator] = 1023 - (adc->adc1->readSingle());

  ch1_pxl[sample_iterator] = map(channel1_raw[sample_iterator],0,1023,0,239);
  ch2_pxl[sample_iterator] = map(channel2_raw[sample_iterator],0,1023,0,239);
  //Serial.printf("\nCH1 RAW VALUE: %d, PIXEL: %d",  channel1_raw[sample_iterator], ch1_pxl[sample_iterator]);
  //Serial.printf("\nCH2 RAW VALUE: %d, PIXEL: %d",  channel2_raw[sample_iterator], ch2_pxl[sample_iterator]);

  adc->startSynchronizedSingleRead(CHANNEL_A, CHANNEL_B);

  if(sample_iterator == 319){
    sampling_timer.end();
    sample_iterator = 0;
  }else{
    sample_iterator++;
  }
  
}

void draw_channel_data(){
  for (int i = 0; i < NUM_SAMPLES; i++) {
    for(int j = 0; j < 239; j++){
      if(ch1_pxl[i] == j){
        im(i, j) = tgx::RGB565_Blue; 
      }else if(ch2_pxl[i] == j){
        im(i, j) = tgx::RGB565_Yellow;
      }else{
        im(i, j) = tgx::RGB565_Black;  
      }
    }
  }
}

void setup(void) {
  Serial.begin(115200);
  sample_iterator = 0;

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
  mathButton.attach(PAUSE_PIN, INPUT_PULLUP);
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
  if (pauseButton.fell()) {
    paused = !paused; // Toggle pause state
    sampling_timer.end();
    Serial.println("PAUSED");
  }
  if(!paused){
    draw_channel_data();
    tft.update(fb);
    sampling_timer.begin(sample, SAMPLING_INTERVAL);
  }
}


