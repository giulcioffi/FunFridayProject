/*
  This example reads audio data from the on-board PDM microphones, and prints
  out the samples to the Serial console. The Serial Plotter built into the
  Arduino IDE can be used to plot the audio data (Tools -> Serial Plotter)

  Circuit:
  - Arduino Nano 33 BLE board

  This example code is in the public domain.
*/
#define iterate 1

#include <PDM.h>

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
short MicValues[256];
int delta = 10;
int counter = 0;
int maxCount = 100;
int avgPower = 0;
int sumVal = 0;
int newVal;
int stepVal;
//int iterate;
int maxVal[iterate];
int minVal[iterate];
int diffMaxMin;
bool computeAvg = false;

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

  for (int i=0; i < iterate; i++){
    maxVal[i] = -65000;
    minVal[i] = 65000;
  }
}

void loop() {
  // wait for samples to be read
  if (samplesRead) {

    for (int i = 0; i < samplesRead; i++) {
      short readVal = sampleBuffer[i];
      MicValues[counter] = readVal;
      //Serial.println(sampleBuffer[i]);
      counter = counter + 1;
      int index = (counter * iterate) / 256 ;
      if (readVal > maxVal[index]) {
        maxVal[index] = readVal;
      }
      if (readVal < minVal[index]) {
        minVal[index] = readVal;
      }
    }


    counter = 0;
    int offset = 0;
    for (int k = 0; k < iterate; k++) {
      int stepCounter = 256/iterate;
      int maxCounter = offset + stepCounter;
      diffMaxMin = maxVal[k] - minVal[k];

      if (diffMaxMin == 0) {
        stepVal = 1;
      } else if (diffMaxMin < 20) {
        computeAvg = true;
        stepVal = 256 / diffMaxMin;
      } else {
        stepVal = 256 / diffMaxMin;
      }
      for (int n = offset; n < maxCounter; n++) {
        counter = counter + 1;
        if (diffMaxMin == 0) {
          newVal = 0;
        } else {
          newVal = (MicValues[n] - minVal[k]) * stepVal;
        }
        if (computeAvg == true) {
          sumVal = sumVal + newVal;
        } else {
          Serial.print(newVal);
          Serial.print(" ");
          Serial.print(0);
          Serial.print(" ");
          Serial.println(255);
          analogWrite(A0, newVal);
        }

      }

      if (computeAvg == true) {
        sumVal = sumVal/stepCounter;
        Serial.print(newVal);
        Serial.print(" ");
        Serial.print(0);
        Serial.print(" ");
        Serial.println(255);
        analogWrite(A0, sumVal);
        computeAvg = false;
      }

      sumVal = 0;
      offset = counter;
    }


    counter = 0;
    for (int i=0; i < iterate; i++){
      maxVal[i] = -65000;
      minVal[i] = 65000;
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
