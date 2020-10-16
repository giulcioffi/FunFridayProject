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
int delta = 10;
int counter = 0;
int maxCount = 100;
int avgPower = 0;
long int AvgValue = 0;
int maxVal = -65000;
int minVal = 65000;
int newVal;
int stepVal;

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
    
    for (int i = 0; i < samplesRead; i++) {
      short readVal = sampleBuffer[i];
      MicValues[counter] = readVal;
      //Serial.println(sampleBuffer[i]);
      counter = counter + 1;
      if (readVal > maxVal) {
        maxVal = readVal;
      }
      if (readVal < minVal) {
        minVal = readVal;
      }
    }
    
    if (counter >= delta) {
      /*
      Serial.println("MIC VALUES:");
      for (int j = 0; j < counter; j++) {
        Serial.println(MicValues[j]);
      }
      */
      int diffMaxMin = maxVal - minVal;
      if (diffMaxMin == 0) {
        stepVal = 1;
      } else {
        stepVal = 256/diffMaxMin;
      }
      /*
      Serial.print("Counter = ");
      Serial.println(counter);
      Serial.print("maxVal = ");
      Serial.println(maxVal);
      Serial.print("minVal = ");
      Serial.println(minVal);
      Serial.print("stepVal = ");
      Serial.println(stepVal);
      Serial.println("Converting into...");
      */
      for (int k = 0; k < counter; k++) {
        if (diffMaxMin == 0) {
          newVal = 0;
        } else {
          newVal = (MicValues[k] - minVal) * stepVal;
        }
        
        Serial.print(newVal);
        Serial.print(" ");
        Serial.print(0);
        Serial.print(" ");
        Serial.println(255);
        analogWrite(A0, newVal);

      }

      counter = 0;
      maxVal = -65000;
      minVal = 65000;
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
/*
void translateValues(int counter) {
  int maxVal = -65000;
  int minVal = 65000;

  for (int j = 0; j < counter; j++) {
    Serial.println(MicValues[j]);
    if (MicValues[j] > maxVal) {
      maxVal = MicValues[j];
    }
    if (MicValues[j] < minVal) {
      minVal = MicValues[j];
    }
  }
  int stepVal = maxVal - minVal;

  Serial.print("Counter = ");
  Serial.println(counter);
  Serial.println("Converting into...");

  for (int k = 0; k < counter; k++) {
    int newVal = (MicValues[k] - minVal) / stepVal;
    Serial.println(newVal);
    analogWrite(A0, newVal);
  }

  counter = 0;
}
*/
