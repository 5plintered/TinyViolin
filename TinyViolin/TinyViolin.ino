

#include "Arduino.h"
#include <Wire.h>
#include <SD.h>
#include <ArduinoSound.h>

#define VOLUME 30
#define FILE_NAME "WOAH.WAV"
#define TPA64_ADDRESS       0x69 // Address of the TPA64 shifted right one bit for wire library
#define THERMOMETER_ADDR  0x0e
#define SENSOR_DATA_START   0x80

#define BUFFER_SIZE 3
#define NUM_SENSORS 64
#define NOISE_THRESHOLD 2 //Adjust this to remove noise in temperature 0 == maximum sensitivity
#define POWER_SWITCH_PIN 0
#define FPSerial Serial1

int frameBuffer[BUFFER_SIZE][NUM_SENSORS];
int sumBuffer[BUFFER_SIZE];
int movementData[NUM_SENSORS];
int frameCounter = BUFFER_SIZE - 1;
bool startMotionDetection = false;
bool debugMode = false;

// variable representing the Wave File
SDWaveFile waveFile;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  InitialiseSD();
  pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);
  
  // start playback
  //----Mp3 play----
  AudioOutI2S.loop(waveFile);
  AudioOutI2S.pause();
}


bool isPaused = true;
void loop()
{
  int powerSwitchVal = digitalRead(POWER_SWITCH_PIN);

  if(powerSwitchVal == LOW) {

    delay(20);

    SaveFrame(frameCounter);
    
    if(frameCounter == 0) startMotionDetection = true; //Start motion detection once buffer is full
    if(startMotionDetection) DetectMotion(frameCounter);
    double movementValue = CalculateAverageMovement();

    if(debugMode) {
      PrintAllFrames();
      PrintSumBuffer();
      Serial.print("Moving Average:");
      Serial.println(movementValue);
    }

    //Advance to next frame
    AdvanceTicker();
    
    if(3 < movementValue && movementValue < 90) {
      if( isPaused) {
        AudioOutI2S.resume();
        isPaused = false;
      }
    }
    else {
      AudioOutI2S.pause();
      isPaused = true;
    }
  }
  else {
    AudioOutI2S.loop(waveFile);
    AudioOutI2S.pause();
    isPaused = true;
  } 

}

void AdvanceTicker() {
  frameCounter--;
  if(frameCounter < 0) {
    frameCounter = BUFFER_SIZE - 1;
  }
}

void InitialiseSD() {
    // setup the SD card, depending on your shield of breakout board
  // you may need to pass a pin number in begin for SS
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // create a SDWaveFile
  waveFile = SDWaveFile(FILE_NAME);

  // check if the WaveFile is valid
  if (!waveFile) {
    Serial.println("wave file is invalid!");
    while (1); // do nothing
  }

  // print out some info. about the wave file
  Serial.print("Bits per sample = ");
  Serial.println(waveFile.bitsPerSample());

  long channels = waveFile.channels();
  Serial.print("Channels = ");
  Serial.println(channels);

  long sampleRate = waveFile.sampleRate();
  Serial.print("Sample rate = ");
  Serial.print(sampleRate);
  Serial.println(" Hz");

  long duration = waveFile.duration();
  Serial.print("Duration = ");
  Serial.print(duration);
  Serial.println(" seconds");

  // adjust the playback volume
  AudioOutI2S.volume(VOLUME);

  // check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    Serial.println("unable to play wave file using I2S!");
    while (1); // do nothing
  }
}

double CalculateAverageMovement() {
  int sumTotal = 0;
  for (int x = 0; x < BUFFER_SIZE; x++ ) {
      sumTotal = sumTotal + sumBuffer[x];
  }
  return sumTotal / BUFFER_SIZE;
}

void PrintSumBuffer() {
  int pixIndex = 0;
  String row = "";
  for (int x = 0; x < BUFFER_SIZE; x++ ) {
      int pix = sumBuffer[x];
      row = row + pix + " ";
  }
}

void DetectMotion(int frame) {
  //Get latest frame
  int currentFrame = frame;
  int lastFrame = currentFrame + 1;
  
  if( currentFrame == BUFFER_SIZE - 1) { //Wrap frames around when calculate buffer edges
    lastFrame = 0;
  }

  int sum = 0;
  //Calculate movement differences
  for(int pixel = 0; pixel < NUM_SENSORS; pixel++) {
    int temperatureDifference = frameBuffer[currentFrame][pixel] - frameBuffer[lastFrame][pixel];
    int movementValue = abs(temperatureDifference);  
    
    if( abs(temperatureDifference) < NOISE_THRESHOLD ) {
      movementValue = 0;
    } 
    
    movementData[pixel] = movementValue;
    sum = sum + movementValue;
  }

  //Save Sum
  sumBuffer[currentFrame] = sum;
}

void SaveFrame(int bufferLocation) {
  int index = bufferLocation % BUFFER_SIZE;
  
  for(int pixel = 0; pixel < NUM_SENSORS; pixel++) {
    frameBuffer[index][pixel] = (int) getPixel(pixel);
  }

}

void PrintFrame(int bufferLocation) {
  int index = bufferLocation % BUFFER_SIZE;
  int pixIndex = 0;
  String row = "";
  for (int x = 0; x < 8; x++ ) {
    for(int y = 0; y < 8; y++) {
      int pix = frameBuffer[bufferLocation][pixIndex];
      row = row + pix + " ";
      pixIndex++;
    }
      row = row + '\n';
  }
  
  Serial.println(row);
}

void PrintAllFrames() {
  for (int x = 0; x < 8; x++ ) {
    String row = "";
    for (int bufferIterator = 0; bufferIterator < BUFFER_SIZE; bufferIterator++) {
      for(int y = 0; y < 8; y++) {
        int pix = frameBuffer[bufferIterator][x*8 + y ];
        row = row + pix + " ";
      }
      row += '\t';
    }
    //Movement Data
    for(int y = 0; y < 8; y++) {
        int pix = movementData[x*8 + y ];
        row = row + pix + " ";
    }

    row += '\t';

    Serial.println(row);
  }
  
}

double getAmbientTemperature() {
  Wire.beginTransmission( TPA64_ADDRESS );
  Wire.write( THERMOMETER_ADDR ); // Address of the pixel we are requesting
  Wire.endTransmission();
  Wire.requestFrom( TPA64_ADDRESS, 2 ); 

  unsigned char lb = Wire.read();
  unsigned char hb = Wire.read();

  int pixel = hb;
  pixel <<= 8;
  pixel |= lb;

  if ( pixel & 0x0800 ) {           // Check the signedness of the value
    pixel = (pixel & 0x07FF) * -1;  // Value is negative, remove sign bit and make value negative  
  }
  return 0.0625 * pixel;
}

double getPixel( int p ) {

  Wire.beginTransmission( TPA64_ADDRESS );
  Wire.write( SENSOR_DATA_START + (2 * p) ); // Address of the pixel we are requesting
  Wire.endTransmission();

  Wire.requestFrom( TPA64_ADDRESS, 2 ); 
  
  // Wait for some data to arrive
  while( Wire.available() < 1 );

  unsigned char lb = Wire.read();
  unsigned char hb = Wire.read();

  int pixel = hb;
  pixel <<= 8;
  pixel |= lb;

  if ( pixel & 0x0800 ) {           // Check the signedness of the value
    pixel = (pixel & 0x07FF) * -1;  // Value is negative, remove sign bit and make value negative  
  }
  
  return 0.25 * pixel;              // Calculate the temperature of the pixel.
    
}

