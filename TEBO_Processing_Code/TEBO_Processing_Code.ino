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
#define PIN_DC      23      // mandatory

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

// Declare ADC object for operation
ADC *adc = new ADC();

// Vars to determine states of operation
bool paused = false;
bool enable_math = false;
bool ENABLE_CUBE = true;

bool enable_channel1 = false;
bool enable_channel2 = false;

bool draw_YCH1 = false;
bool draw_YCH2 = false;
bool adjust_trigger = false;

// Default positions for cursors
int cursor1_ypos = 512;
int cursor1_xpos = 768;
int cursor2_ypos = 513;
int cursor2_xpos = 256;

// Encoder position vars
long preserved_x1pos = 0;
long preserved_y1pos = 0;
long preserved_x2pos = 0;
long preserved_y2pos = 0;

// ADC Running conditions and buffers for the two channels
#define NUM_SAMPLES 3200
int sample_iterator;
int channel1_raw[NUM_SAMPLES];
int channel2_raw[NUM_SAMPLES];
int ch1_pxl[NUM_SAMPLES];
int ch2_pxl[NUM_SAMPLES];

#define CHANNEL_A   14
#define CHANNEL_B   19
#define ADC_RESOLUTION    10      // Resolution in bits
#define ADC_OVERSAMPLING  0      // 
#define SAMPLING_INTERVAL 1      // microseconds
IntervalTimer sampling_timer;

volatile float time_scale = 1.0;
long preserved_time_encoder = 0;

// Encoder pin definitions
#define ROT_1A 2  // LEFT Encode Clockwise
#define ROT_1B 3  // LEFT Encode Counter-Clockwise
#define ROT_1C 4  // LEFT Encode Button
#define ROT_2A 5  // RIGHT Encode Clockwise
#define ROT_2B 6  // RIGHT Encode Counter-Clockwise
#define ROT_2C 7  // RIGHT Encode Button
#define PAUSE_PIN 9 // Bottom Button
#define MATH_PIN 22 // Top Button

// Bounce objects for encoders and push buttons
Bounce pauseButton = Bounce();
Bounce mathButton = Bounce();
Bounce encoderAButton = Bounce();
Bounce encoderBButton = Bounce();

// Trigger conditions
#define TRIGGER_MIN 0
#define TRIGGER_MAX 1023
#define TRIGGER_STEP 10  // Step size per encoder tick
#define TRIGGER_TIMEOUT 1000 

Encoder encoderA(ROT_1A, ROT_1B);
Encoder encoderB(ROT_2A, ROT_2B);

// Voltage/vertical scale conditions and encoder values
#define SCALE_MIN 0.1
#define SCALE_MAX 5.0
#define SCALE_STEP 0.1
volatile float vertical_scale = 1.0;
long preserved_voltage_encoder = 0;

// Additional trigger conditions
bool triggered = false;
volatile int trigger_level = 512;
volatile bool show_trigger = false;
unsigned long last_encoder_activity = 0;
long preserved_trigger_encoder = 0;

// Statements needed for cube/loading screen functionality
const float ratio = ((float)SLX) / SLY; // aspect ratio
uint16_t zbuf[SLX * SLY];  
const int tex_size = 128;
RGB565 texture_data[tex_size*tex_size];
Image<RGB565> texture(texture_data, tex_size, tex_size);
const Shader LOADED_SHADERS = SHADER_ORTHO | SHADER_PERSPECTIVE | SHADER_ZBUFFER | SHADER_FLAT | SHADER_TEXTURE_BILINEAR | SHADER_TEXTURE_WRAP_POW2;
Renderer3D<RGB565, LOADED_SHADERS, uint16_t> renderer;


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

void update_time_scaling() {
  long new_position = encoderA.read() / 4;  // Adjust sensitivity

  if (new_position != preserved_time_encoder) {
    preserved_time_encoder = new_position;
    time_scale = constrain(time_scale + (new_position * 0.1), 0.1, 10.0); // Adjust the range as needed
    Serial.printf("Time Scale: %.1fx\n", time_scale);
  }
}


void sample(){

  while(adc->adc0->isConverting() || adc->adc1->isConverting());

  int ch1_value = 1023 - adc->adc0->readSingle();
  int ch2_value = 1023 - adc->adc1->readSingle();

  // TRIGGER TEST
  if (!triggered) {
    // Check if the trigger condition is met
    if ((ch1_value > trigger_level && channel1_raw[(sample_iterator - 1 + NUM_SAMPLES) % NUM_SAMPLES] <= trigger_level) || (ch2_value > trigger_level && channel2_raw[(sample_iterator - 1 + NUM_SAMPLES) % NUM_SAMPLES] <= trigger_level)){
      triggered = true;
      sample_iterator = 0; // Reset the sample iterator when trigger is fired
    }
  }

  channel1_raw[sample_iterator] = ch1_value;
  channel2_raw[sample_iterator] = ch2_value;

  ch1_pxl[sample_iterator] = map(channel1_raw[sample_iterator],0,1023,0,239);
  ch2_pxl[sample_iterator] = map(channel2_raw[sample_iterator],0,1023,0,239);

  adc->startSynchronizedSingleRead(CHANNEL_A, CHANNEL_B);

  if(sample_iterator == (NUM_SAMPLES - 1)){
    //sampling_timer.end();
    sample_iterator = 0;
    triggered = false;  // TRIGGER TEST
    return;
  }else{
    sample_iterator++;
    sample();
  }
  
}

void updateY_cursorA() {
  long new_position = encoderA.read() / 4;  // Adjust sensitivity

  if (new_position != preserved_y1pos) {
    preserved_y1pos = new_position;
    cursor1_ypos = constrain(512 + (new_position * 10), 0, 1023);
  }
}
void updateX_cursorA() {
  long new_position = encoderA.read() / 4;  // Adjust sensitivity

  if (new_position != preserved_x1pos) {
    preserved_x1pos = new_position;
    cursor1_xpos = constrain(512 + (new_position * 10), 0, 1023);
  }
}
void updateY_cursorB() {
  long new_position = encoderB.read() / 4;  // Adjust sensitivity

  if (new_position != preserved_y2pos) {
    preserved_y2pos = new_position;
    cursor2_ypos = constrain(512 + (new_position * 10), 0, 1023);
  }
}
void updateX_cursorB() {
  long new_position = encoderB.read() / 4;  // Adjust sensitivity

  if (new_position != preserved_x2pos) {
    preserved_x2pos = new_position;
    cursor2_xpos = constrain(512 + (new_position * 10), 0, 1023);
  }
}

// Handles cursor positions and displayed differences in voltage/time
void draw_math_functions(){
  if(!enable_math){
    return;
  }
  if(encoderAButton.fell()){
    draw_YCH1 = !draw_YCH1;
    if(draw_YCH1){
      encoderA.write(preserved_y1pos * 4);
    }else{
      encoderA.write(preserved_x1pos * 4);
    }
  }
  if(encoderBButton.fell()){
    draw_YCH2 = !draw_YCH2;
    if(draw_YCH2){
      encoderB.write(preserved_y2pos * 4);
    }else{
      encoderB.write(preserved_x2pos * 4);
    }
  }

  if(draw_YCH1){
    updateY_cursorA();
  }else{
    updateX_cursorA();
  }
  if(draw_YCH2){
    updateY_cursorB();
  }else{
    updateX_cursorB();
  }
  

  int CH1_Y = map(cursor1_ypos, 0, 1023, 239, 0);
  int CH1_X = map(cursor1_xpos, 0, 1023, 319, 0);
  int CH2_Y = map(cursor2_ypos, 0, 1023, 239, 0);
  int CH2_X = map(cursor2_xpos, 0, 1023, 319, 0);

  for (int x = 0; x < SLX; x++) {
    im(x, CH1_Y) = tgx::RGB565_Red;
    im(x, CH2_Y) = tgx::RGB565_Blue;
  }
  for (int y = 0; y < SLY; y++){
    im(CH1_X, y) = tgx::RGB565_Red;
    im(CH2_X, y) = tgx::RGB565_Blue;
  }

  float voltage_diff_analog = abs(cursor1_ypos - cursor2_ypos);
  float voltage_diff_premap = map(voltage_diff_analog, 0, 1023, 0, 3.3);
  float voltage_diff_postmap = map(voltage_diff_premap, 0, 3.3, 0, 10.0);
  float voltage_diff_postmap_scaled = abs(voltage_diff_postmap * (1.0 / vertical_scale));

  tgx::iVec2 anchor_point;

  char MathTextA[30]; 
  anchor_point = { 280, 0 };
  snprintf(MathTextA, sizeof(MathTextA), "dV: %.3f V", voltage_diff_postmap_scaled);
  im.drawTextEx(MathTextA, anchor_point, font_tgx_OpenSans_12, tgx::Anchor::CENTERTOP, true, true, tgx::RGB565_White);

  float time_diff_analog = abs(cursor1_xpos - cursor2_xpos);
  float time_diff_premap = map(time_diff_analog, 0, 1023, 0, 319);
  long long time_diff_divs = map(time_diff_premap, 0, 319, 0, 16.0000000);

  double Tdiv = (1.0/time_scale)*0.00002469; 
  double time_diff_postmap_scaled = Tdiv * time_diff_divs;
  float  freqDiv= 1.0/float(time_diff_postmap_scaled);

    char prefix[3];

        //micro
          if  (freqDiv > 10000){
          strcpy(prefix, "u");
          time_diff_postmap_scaled = time_diff_postmap_scaled * 1000000;

          // milli
          }else if (10000 > freqDiv || freqDiv > 100){ 
          strcpy(prefix, "m");
          time_diff_postmap_scaled = time_diff_postmap_scaled * 1000.0;

          //none
          }else if (100 > freqDiv){
          strcpy(prefix, "");
    }

    // display seconds per division
  char MathTextB[20];  
  anchor_point = { 280, 20 };
  snprintf(MathTextB, sizeof(MathTextB), "dT: %.3f%ss", time_diff_postmap_scaled, prefix);
  im.drawTextEx(MathTextB, anchor_point, font_tgx_OpenSans_12, tgx::Anchor::CENTERTOP, true, true, tgx::RGB565_White);
  
}

// Draws grid and alters time+volts per div according to scale values
void display_scales() {
    im.drawRect({ 0, 0, 320, 20}, tgx::RGB565_White);
    tgx::iVec2 anchor_point;

    char timeScaleText[20];  
    snprintf(timeScaleText, sizeof(timeScaleText), "Time: %.1fx", time_scale);
    anchor_point = { 40, 20 };
    im.drawTextEx(timeScaleText, anchor_point, font_tgx_OpenSans_12, tgx::Anchor::CENTERTOP, true, true, tgx::RGB565_White);

    char verticalScaleText[30]; 
    anchor_point = { 40, 0 };
    snprintf(verticalScaleText, sizeof(verticalScaleText), "Vertical: %.1fx", vertical_scale);
    im.drawTextEx(verticalScaleText, anchor_point, font_tgx_OpenSans_12, tgx::Anchor::CENTERTOP, true, true, tgx::RGB565_White);

    char triggerValueText[30]; 
    anchor_point = { 40, 40 };
    snprintf(triggerValueText, sizeof(triggerValueText), "Trigger: %.1dx", trigger_level);
    im.drawTextEx(triggerValueText, anchor_point, font_tgx_OpenSans_12, tgx::Anchor::CENTERTOP, true, true, tgx::RGB565_White);

    im.drawFastVLine({ 160, 0}, 240, tgx::RGB565_White, 20.0f);// draw a vertical line
    im.drawFastHLine({ 0, 120}, 320, tgx::RGB565_White, 20.0f);// draw an horizontal line

    for (int i = 0; i < 16; i++) {
        im.drawFastVLine({ i * 20, 115}, 10, tgx::RGB565_White, 20.0f);// draw a vertical line
        im.drawFastHLine({ 155, i * 20}, 10, tgx::RGB565_White, 20.0f);// draw a vertical line
    }


    // draw the current seconds per division & voltage per division

    float vdiv_nominal = (10.0 / 12.0);
    float Vdiv = vdiv_nominal * (1.0/vertical_scale) * 1.42;

    double Tdiv = (1.0/time_scale)*0.00002469; 

    float  freqDiv = 1.0/float(Tdiv);

    char prefix[3];

        //micro
          if  (freqDiv > 10000){
          strcpy(prefix, "u");
          Tdiv = Tdiv * 1000000;

          // milli
          }else if (10000 > freqDiv || freqDiv > 100){ 
          strcpy(prefix, "m");
          Tdiv = Tdiv * 1000.0;

          //none
          }else if (100 > freqDiv){
          strcpy(prefix, "");
    }

    // display seconds per division
    char timeDivText[20];  
    snprintf(timeDivText, sizeof(timeDivText), "Tdiv: %.2f%ss", Tdiv, prefix);
    anchor_point = { 40, 230 };
    im.drawTextEx(timeDivText, anchor_point, font_tgx_OpenSans_12, tgx::Anchor::CENTERBOTTOM, true, true, tgx::RGB565_White);
    
    // display volts per division
    char verticalDivText[30]; 
    anchor_point = { 110, 230 };
    snprintf(verticalDivText, sizeof(verticalDivText), "Vdiv: %.2fV", Vdiv);
    im.drawTextEx(verticalDivText, anchor_point, font_tgx_OpenSans_12, tgx::Anchor::CENTERBOTTOM, true, true, tgx::RGB565_White);
}

// Main drawing function to parse samples onto the display, taking trigger and volt/time scale into account. 
void draw_channel_data(){
 
  im.clear(tgx::RGB565_Black);

  if(!enable_math){
    update_time_scaling();
    display_scales();
  }
  if(!enable_math && adjust_trigger){
    update_trigger();
  }
  int trigger_pixel = map(trigger_level, 0, 1023, 239, 0);
  if(!enable_math && !adjust_trigger){
    update_voltage_scaling();
  }


  int trigger_index = -1;

  // Find the index where the trigger condition is met
  for (int i = 1; i < NUM_SAMPLES; i++) {
    if ((channel1_raw[i] > trigger_level && channel1_raw[i - 1] <= trigger_level) || (channel2_raw[i] > trigger_level && channel2_raw[i - 1] <= trigger_level)) {
      trigger_index = i;
      break;
    }
  }
  
  // Draw only values in the trigger if set
  if (trigger_index != -1) {
    for (int i = 0; i < NUM_SAMPLES; i++) {
      
      int index = (trigger_index + i) % NUM_SAMPLES;
      int ch1_scaled = constrain((ch1_pxl[index] - 120) * vertical_scale + 120, 0, 239);
      int ch2_scaled = constrain((ch2_pxl[index] - 120) * vertical_scale + 120, 0, 239);
      int x = i * time_scale;
      if (x >= SLX) break; // Stop drawing if x exceeds screen width
      if(enable_channel1){
        im(x, ch1_scaled) = tgx::RGB565_Blue;
      }
      if(enable_channel2){
        im(x, ch2_scaled) = tgx::RGB565_Yellow;
      }

      // Interpolation for high time scales(Unused in current form)
      /*if (time_scale > 2.0) {
        int next_index = (index + 1) % NUM_SAMPLES;
        int next_x = (i + 1) * time_scale;
        if (next_x < SLX) {
          for (int j = x; j < next_x; j++) {
            int interp_ch1prescale = ch1_pxl[index] + (ch1_pxl[next_index] - ch1_pxl[index]) * (j - x) / (next_x - x);
            int interp_ch2prescale = ch2_pxl[index] + (ch2_pxl[next_index] - ch2_pxl[index]) * (j - x) / (next_x - x);
            
            int interp_ch1 = constrain((interp_ch1prescale - 120) * vertical_scale + 120, 0, 239);
            int interp_ch2 = constrain((interp_ch2prescale - 120) * vertical_scale + 120, 0, 239);

            if(enable_channel1){
              im(j, interp_ch1) = tgx::RGB565_Blue;
            }
            if(enable_channel2){
              im(j, interp_ch2) = tgx::RGB565_Yellow;
            }
          }
        }
      }*/


    }
  }else{ // Free/no trigger mode
    for (int i = 0; i < NUM_SAMPLES; i++) {
      
      int ch1_scaled = constrain((ch1_pxl[i] - 120) * vertical_scale + 120, 0, 239);
      int ch2_scaled = constrain((ch2_pxl[i] - 120) * vertical_scale + 120, 0, 239);
      int x = i * time_scale;
      if (x >= SLX) break; // Stop drawing if x exceeds screen width
      if(enable_channel1){
        im(x, ch1_scaled) = tgx::RGB565_Blue;
      }
      if(enable_channel2){
        im(x, ch2_scaled) = tgx::RGB565_Yellow;
      }
    }
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
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setAveraging(ADC_OVERSAMPLING);
  adc->adc1->setResolution(ADC_RESOLUTION);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
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

  // Cube/loading screen setup
  renderer.setViewportSize(SLX,SLY);
  renderer.setOffset(0, 0);
  renderer.setImage(&im);
  renderer.setZbuffer(zbuf);
  renderer.setCulling(1);
  renderer.setTextureQuality(SHADER_TEXTURE_BILINEAR);
  renderer.setTextureWrappingMode(SHADER_TEXTURE_WRAP_POW2);
  renderer.setShaders(SHADER_FLAT | SHADER_TEXTURE );
  renderer.setPerspective(45, ratio, 1.0f, 100.0f);
  texture.fillScreen(RGB565_Blue);


  adc->startSynchronizedSingleRead(CHANNEL_A, CHANNEL_B); // start ADC, read A0 and A1 channels
  //sampling_timer.begin(sample, SAMPLING_INTERVAL);
  Serial.println("SETUP DONE");
}


// Cube 2D draw function
elapsedMillis em = 0; // time
int nbf = 0; ; // number frames drawn
int projtype = 0; // current projection used. 
void splash()
    {
    static int count = 0;    
    static RGB565 color;
    if (count == 0)
        color = RGB565((int)random(32), (int)random(64), (int)random(32)); 
    count = (count + 1) % 400;
    iVec2 pos(random(tex_size), random(tex_size));
    int r = random(10);
    texture.drawRect(iBox2( pos.x - r, pos.x + r, pos.y - r, pos.y + r ), color);
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


  if(encoderAButton.fell() && !enable_math){
    if(!enable_channel1 && !enable_channel2 && ENABLE_CUBE){
      ENABLE_CUBE = false;
    }else if(!enable_channel1 && !enable_channel2 && !ENABLE_CUBE){
      enable_channel1 = true;
    }else if(enable_channel1 && !enable_channel2){
      enable_channel2 = true;
    }else if(enable_channel1 && enable_channel2){
      enable_channel1 = false;
    }else if(!enable_channel1 && enable_channel2){
      enable_channel2 = false;
      ENABLE_CUBE = true;
    }
  }


  if(pauseButton.fell()) {
    paused = !paused; // Toggle pause state
    //sampling_timer.end();
    Serial.printf("\nPAUSE Status: %d", paused);
  }


  if(mathButton.fell()){
    enable_math = !enable_math;
    if(enable_math){
      if(draw_YCH1){
        encoderA.write(preserved_y1pos * 4);
      }else{
        encoderA.write(preserved_x1pos * 4);
      }
      if(draw_YCH2){
        encoderB.write(preserved_y2pos * 4);
      }else{
        encoderB.write(preserved_x2pos * 4);
      }
    }else{
      if(adjust_trigger){
        encoderB.write(preserved_trigger_encoder * 4);
      }else{
        encoderB.write(preserved_voltage_encoder * 4);
      }
      encoderA.write(preserved_time_encoder * 4);
    }
  }

  if(ENABLE_CUBE){
    // model matrix
    fMat4 M;
    M.setRotate(em / 11.0f, { 0,1,0 });
    M.multRotate(em / 23.0f, { 1,0,0 });
    M.multRotate(em / 41.0f, { 0,0,1 });
    M.multTranslate({ 0, 0, -5 });

    im.fillScreen((projtype) ? RGB565_Black : RGB565_Gray); // erase the screen, black in perspective and grey in orthographic projection
    renderer.clearZbuffer(); // clear the z buffer        
    renderer.setModelMatrix(M);// position the model

    renderer.drawCube(&texture, & texture, & texture, & texture, & texture, & texture); // draw the textured cube


    // info about the projection type
    im.drawText((projtype) ? "TEBO Program by Jack Cunningham, Jeremy Geno, Nick Alves" : "Praise The Cube!", {3,12 }, font_tgx_OpenSans_Bold_10, RGB565_Red);

    // add fps counter
    //tft.overlayFPS(fb); 
    
    // update the screen (async). 
    tft.update(fb);

    // add a random rect on the texture.
    splash();

    // switch between perspective and orthogonal projection every 1000 frames.
    if (nbf++ % 1000 == 0){
        projtype = 1 - projtype;

        if (projtype)
            renderer.setPerspective(45, ratio, 1.0f, 100.0f);
        else
            renderer.setOrtho(-1.8 * ratio, 1.8 * ratio, -1.8, 1.8, 1.0f, 100.0f);

        tft.printStats();
        diff1.printStats();
        diff2.printStats();
        }

  }


  if(!paused && !ENABLE_CUBE){
    sample();
    draw_channel_data();
    draw_math_functions();
    tft.update(fb);
    //sampling_timer.begin(sample, SAMPLING_INTERVAL);
  }
}


