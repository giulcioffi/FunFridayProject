/*
  This example reads audio data from the on-board PDM microphones, and prints
  out the samples to the Serial console. The Serial Plotter built into the
  Arduino IDE can be used to plot the audio data (Tools -> Serial Plotter)

  Circuit:
  - Arduino Nano 33 BLE board

  This example code is in the public domain.
*/

#include <PDM.h>

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
short MicValues[256];
int counter = 0;
int lastReadVal = 0;
bool firstHalf = true;
bool secondHalf = false;
int freq = 0;
int keepUp = 0;
int keepLow = 0;

// number of samples read
volatile int samplesRead;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // configure the data receive callback
  PDM.onReceive(onPDMdata);

  // optionally set the gain, defaults to 20
  // PDM.setGain(30);

  // initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
  
}

void loop() {
  // wait for samples to be read
  if (samplesRead) {

    // print samples to the serial monitor or plotter
    for (int i = 0; i < samplesRead; i++) {
      //Serial.println(sampleBuffer[i]);
      short readVal = sampleBuffer[i];
      counter = counter + 1;

      if (secondHalf) {
        if (readVal > 0) {
          keepUp = keepUp +1;
          if (keepUp == 5) {
            freq = freq + 1;
            firstHalf = true;
            secondHalf = false;
          }
        } else {
          keepUp = 0;
        }
      } else {
        if (readVal < 0) {
          keepLow = keepLow + 1;
          if (keepLow == 5) {
            secondHalf = true;
            firstHalf = false;
          }
        } else {
          keepLow = 0;
        }
      }

      if (counter == 16000) {
        Serial.println(freq);
        freq = 0;
        counter = 0;
      }
      
    }

    // clear the read count
    samplesRead = 0;
  }
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
