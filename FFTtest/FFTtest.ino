/*
  This example reads audio data from the on-board PDM microphones, and prints
  out the samples to the Serial console. The Serial Plotter built into the
  Arduino IDE can be used to plot the audio data (Tools -> Serial Plotter)

  Circuit:
  - Arduino Nano 33 BLE board

  This example code is in the public domain.
*/

#include <PDM.h>
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

#define N_iterations 16
#define N_samples 256

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[N_samples];
double MicValues[N_samples*(N_iterations+1)];
int delta = 10;
int counter = 0;
int maxCount = 100;
int avgPower = 0;
int sumVal = 0;
int newVal;
double stepVal = 1;
int iteration = 0;
double maxVal = 0;
int minVal;
int lastMaxVal=65000;
int lastMinVal=-65000;
int diffMaxMin;
bool inRange = false;
int counterPositive= 0;
bool startMic = true;

// number of samples read
volatile int samplesRead;

const double samplingFrequency = 16000;
const uint16_t samples = N_samples; //This value MUST ALWAYS be a power of 2

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
  if (!PDM.begin(1, 16000)) {   //16000
    Serial.println("Failed to start PDM!");
    while (1);
  }
}

void loop() {
  // wait for samples to be read
  
  if (samplesRead) {
    /*
    counterPositive = 0;
    int offset = 0;
    inRange = false;

    maxVal = -65000;
    minVal = 65000;
    sumVal = 0;
    */

    for (int i = 0; i < samplesRead; i++) {
      short readVal = sampleBuffer[i];
      
      MicValues[counter] = (double)readVal;
      //Serial.println(sampleBuffer[i]);
      /*
      if (readVal > maxVal) {
        maxVal = readVal;
      }
      if (readVal < minVal) {
        minVal = readVal;
      }
      */
      
      counter = counter + 1;
    }

    /*
    diffMaxMin = maxVal - minVal;

    if (diffMaxMin == 0) {
      stepVal = 1;
    } else if (diffMaxMin <= 50) {
      inRange = true;
      if (lastMinVal < minVal) {
        minVal = lastMinVal;
      }
      if (lastMaxVal > maxVal) {
        maxVal = lastMaxVal;
      }
      diffMaxMin = maxVal - minVal;
      stepVal = 256 / diffMaxMin;
    } else {
      stepVal = 256 / diffMaxMin;
    }

    for (int n = 0; n < counter; n++) {
      if (diffMaxMin == 0) {
        newVal = (lastMaxVal + lastMinVal)/2;
        putMicValue(newVal, 0, 255);
        break;
      } else {
        newVal = (MicValues[n] - minVal) * stepVal;
        putMicValue(newVal, 0, 255);
      }

    }

    // clear the read count
    samplesRead = 0;

    if (diffMaxMin != 0) {
      updateLastMaxMin(maxVal, minVal);
    }

    */
  }
  
  if (counter >= (N_samples*N_iterations)) {
    //PrintVector(MicValues, (counter >> 1));
    double fftVal = FFT.MajorPeak(MicValues, samples, samplingFrequency);
    //Serial.println(fftVal, 6);
    computeLEDval(fftVal);
    counter = 0;
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

void putMicValue(int val, int lowRange, int upRange) {
  Serial.print(val);
  Serial.print(" ");
  Serial.print(lowRange);
  Serial.print(" ");
  Serial.println(upRange);
  analogWrite(A0, val);
}

void updateLastMaxMin(int maxVal, int minVal) {
  lastMaxVal = maxVal;
  lastMinVal = minVal;
}

void PrintVector(double *vData, uint16_t bufferSize)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa = ((i * 1.0 * samplingFrequency) / samples);
    /*
    Serial.println(abscissa, 6);
    Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
    */
  }
  //Serial.println();
}

void computeLEDval(double fftVal)
{
  if (fftVal > maxVal) {
    maxVal = fftVal;
  }
  if (maxVal != 0) {
    stepVal = 256 / maxVal;
  }
  newVal = (int)(fftVal * stepVal);
  putMicValue(newVal, 0, 255);
}
