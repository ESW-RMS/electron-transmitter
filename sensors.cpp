/*

  Stanford Engineers for a Sustainable World
  Remote Monitoring System | August 2018
  Brian Tanabe | btanabe@stanford.edu

  File: sensors.cpp
  --------------------------
  Implementation of sensors.h

*/
#include "application.h"
#include <cmath>
#include "sensors.h"
#include <stdlib.h>

Sensors::Sensors() {

}

void Sensors::init() {

  for(unsigned int i = 0; i < input_count; i++) {
    pinMode(input[i].pin, INPUT);
  }
  input[0].pin = A0;
  input[0].periodMin = 15000;
  input[0].periodMax = 25000;
  input[0].xShiftMin = 0;
  input[0].xShiftMax = 12500;
  input[0].amplitudeMin = 1;
  input[0].amplitudeMax = 409500;
  input[0].yShift = -330;
  input[0].waveMin = 800;
  input[0].waveMax = 4096;
  input[0].fa = 0;
  input[0].fb = 0;
  input[0].fc = 1;
  input[0].fd = 0;
  input[0].a = 0;
  input[0].b = 0;
  input[0].c = 1;
  input[0].d = 0;
  input[0].rectified = true;

  // Current 1
  input[1].pin = A1;
  input[1].periodMin = 15000;
  input[1].periodMax = 25000;
  input[1].xShiftMin = 0;
  input[1].xShiftMax = 12500;
  input[1].amplitudeMin = 1;
  input[1].amplitudeMax = 409500;
  input[1].yShift = 1975;
  input[1].waveMin = -1;
  input[1].waveMax = 4096;
  input[1].fa = 0;
  input[1].fb = 0;
  input[1].fc = 1;
  input[1].fd = 0;
  input[1].a = 0;
  input[1].b = 0;
  input[1].c = 1;
  input[1].d = 0;
  input[1].rectified = true;

  // Current 2
  input[2].pin = A2;
  input[2].periodMin = 15000;
  input[2].periodMax = 25000;
  input[2].xShiftMin = 0;
  input[2].xShiftMax = 12500;
  input[2].amplitudeMin = 1;
  input[2].amplitudeMax = 409500;
  input[2].yShift = 2400;
  input[2].waveMin = -1;
  input[2].waveMax = 4096;
  input[2].fa = 0;
  input[2].fb = 0;
  input[2].fc = 1;
  input[2].fd = 0;
  input[2].a = 0;
  input[2].b = 0;
  input[2].c = 1;
  input[2].d = 0;
  input[2].rectified = true;

  // Current 3
  input[3].pin = A3;
  input[3].periodMin = 15000;
  input[3].periodMax = 25000;
  input[3].xShiftMin = 0;
  input[3].xShiftMax = 12500;
  input[3].amplitudeMin = 1;
  input[3].amplitudeMax = 409500;
  input[3].yShift = 1975;
  input[3].waveMin = -1;
  input[3].waveMax = 4096;
  input[3].fa = 0;
  input[3].fb = 0;
  input[3].fc = 1;
  input[3].fd = 0;
  input[3].a = 0;
  input[3].b = 0;
  input[3].c = 1;
  input[3].d = 0;
  input[3].rectified = true;

}

void Sensors::refreshAll() {
    #ifdef DEBUG1
  for(int i = 5; i > 0; i--) {
    Serial.println(String::format("Starting measurement in %d", i));
    delay(1000);
  }
    #endif
  for(int attempts = 0; attempts < maxMeasurementAttempts; attempts++) {

        #ifdef DEBUG1
    Serial.println(String::format("--------------------Measurement attempt %d--------------------", attempts+1));
    int startTime = -millis();
        #endif

    recordSamples();
        #ifdef DEBUG1
    Serial.println("Samples recorded");
        #endif

    if(checkStatus()) {

      analyzeSmoothedWaves();
            #ifdef DEBUG1
      Serial.println("Preliminary analysis complete");
            #endif
      bruteforceFrequencies();
            #ifdef DEBUG1
      Serial.println("Frequency analysis complete");
            #endif
      bruteforceAmplitudes();
            #ifdef DEBUG1
      Serial.println("Wave analysis complete");
            #endif
    } else {
            setOutputs(0); // Set currents to 0 if generator is off
          }

        #ifdef DEBUG1
          Serial.println("\n---------RESULTS---------");
          Serial.println(String::format("Voltage Error - %f", input[0].error));
          Serial.println(String::format("Current 1 Error - %f", input[0].error));
          Serial.println(String::format("Current 2 Error - %f", input[0].error));
          Serial.println(String::format("Current 3 Error - %f", input[0].error));
          Serial.println("-------------------------");
          if(input[0].error < maxError) {
            Serial.println(String::format("Voltage: %d", input[0].amplitude));
            Serial.println(String::format("Frequency: %f", input[0].frequency));
          } else {
            Serial.println("Voltage inconclusive");
            Serial.println("Frequency inconclusive");
          }
          if(input[1].error < maxError) {
            Serial.println(String::format("Current 1: %d", input[1].amplitude));
          } else {
            Serial.println("Current 1 inconclusive");
          }
          if(input[2].error < maxError) {
            Serial.println(String::format("Current 2: %d", input[2].amplitude));
          } else {
            Serial.println("Current 2 inconclusive");
          }
          if(input[3].error < maxError) {
            Serial.println(String::format("Current 3: %d", input[3].amplitude));
          } else {
            Serial.println("Current 3 inconclusive");
          }
          Serial.println(String::format("Time to measure: %f seconds", (double)(startTime + millis()) / 1000.));
          Serial.println("------------------\n");
        #endif

          if(measurementsValid) {
            voltage = input[0].rms*compressionMultiplier;
            frequency = input[0].frequency*compressionMultiplier;
            current_1 = input[1].rms*compressionMultiplier;
            current_2 = input[2].rms*compressionMultiplier;
            current_3 = input[3].rms*compressionMultiplier;

            #ifdef DEBUG2
            Serial.println("\n---------FINAL OUTPUT---------");
            Serial.println(String::format("Voltage RMS: %f", (double)voltage/(double)compressionMultiplier));
            Serial.println(String::format("Frequency: %f", (double)frequency/(double)compressionMultiplier));
            Serial.println(String::format("Current 1 RMS: %f", (double)current_1/(double)compressionMultiplier));
            Serial.println(String::format("Current 2 RMS: %f", (double)current_2/(double)compressionMultiplier));
            Serial.println(String::format("Current 3 RMS: %f", (double)current_3/(double)compressionMultiplier));
            Serial.println("------------------\n");
            #endif

            break;

          } else if(attempts == maxMeasurementAttempts - 1) {
            voltage = invalidPlaceholder;
            frequency = invalidPlaceholder;
            current_1 = invalidPlaceholder;
            current_1 = invalidPlaceholder;
            current_1 = invalidPlaceholder;
          }
        }
      }

      void Sensors::refreshStatus() {
        for(unsigned int i = 0; i < status_samples; i++) {
          samples[0][i] = analogRead(input[0].pin);
        }
        inputActive = checkStatus();
      }

      void Sensors::fieldTest() {
        while(true) {
          refreshAll();
        }
      }

      bool Sensors::getGeneratorStatus() {
        return inputActive;
      }

      unsigned short Sensors::getVoltage() {
        return voltage;
      }

      unsigned short Sensors::getFrequency() {
        return frequency;
      }

      unsigned short Sensors::getCurrent_1() {
        return current_1;
      }

      unsigned short Sensors::getCurrent_2() {
        return current_2;
      }

      unsigned short Sensors::getCurrent_3() {
        return current_3;
      }

      double Sensors::waveError(int measurementIndex, int iterator, int xShift, int amplitude, int period) {
        return pow((samples[measurementIndex][iterator] - simulateWave(input[measurementIndex].yShift, input[measurementIndex].rectified, xShift, input[measurementIndex].xShiftMax, amplitude, iterator, period)), 2);
      }

      double Sensors::simulateWave(int yShift, bool rectified, int xShift, int xShiftMax, int amplitude, int iterator, int period) {
        if(rectified) {
          return ((double)yShift + ((double)amplitude/(double)100) * fabs(cos( 2.0 * pi *  (((double)xShift/(double)xShiftMax) + (((double)iterator*measurementDuration) / (double)period)))));
        } else {
          return ((double)yShift + ((double)amplitude/(double)100) * cos( 2.0 * pi *  (((double)xShift/(double)xShiftMax) + (((double)iterator*measurementDuration) / (double)period))));
        }
      }

      void Sensors::recordSamples() {
    // Record samples and sampling time
    #ifndef DEBUG3
        int sampleTime = -micros();
        for(unsigned int i = 0; i < measurement_samples; i++) {
          for(unsigned int j = 0; j < input_count; j++) {
            samples[j][i] = analogRead(input[j].pin);
          }
        }
        sampleTime += micros();
        measurementDuration = (sampleTime / measurement_samples);
    #else
        measurementDuration = 160;
        srand(micros() % 1000000);
    for(int a = 0; a < 2; a++) { // The first iteration is to calibrate measurementDuration
        int p = 18000 + rand() % 4000; // Random period 18000-22000
        if(a > 0) {
          Serial.println("\n---------GENERATED SAMPLES---------");
          Serial.println(String::format("Period: %d", p));
        }
        for(unsigned int i = 0; i < input_count; i++) {
          input[i].xShift = rand() % (input[i].periodMax / 2);
          input[i].amplitude = 20 + rand() % input[i].amplitudeMax;
          if(a > 0) {
            Serial.println(String::format("%d - xShift: %d", i, input[i].xShift));
            Serial.println(String::format("%d - amplitude: %d", i, input[i].amplitude));
          }
        }
        if(a > 0) {
          Serial.println("------------------");
        }
        int sampleTime = -micros();
        for(unsigned int i = 0; i < measurement_samples; i++) {
          for(unsigned int j = 0; j < input_count; j++) {
            samples[j][i] = simulateWave(input[j].yShift, input[j].rectified, input[j].xShift, input[j].xShiftMax, input[j].amplitude, i, p);
          }
        }
        if(a < 1) {
          sampleTime += micros();
          measurementDuration = (sampleTime / measurement_samples);
        }
      }

    #endif
    }

void Sensors::analyzeSmoothedWaves(int measurementIndex/* = -1*/) {
    #ifdef DEBUG4
    Serial.println("------------------");
    #endif
    unsigned int index;
    unsigned int length;
    if(measurementIndex == -1) {
      index = 0;
      length = input_count;
    } else {
      index = measurementIndex;
      length = measurementIndex + 1;
    }
    for(; index < length; index++) {
      double avg = 0;
      for(unsigned int i = 0; i < smoothing_n; i++) {
        avg += samples[index][i];
      }
      smoothed_wave[0] = avg/smoothing_n;
      double max = smoothed_wave[0];
      int maxIndex = 0;
      double min = smoothed_wave[0];
      for(int i = 0, len = measurement_samples-smoothing_n; i < len; i++) {
        avg -= samples[index][i];
        avg += samples[index][i+smoothing_n];
        smoothed_wave[i + 1] = avg/smoothing_n;
        if(smoothed_wave[i + 1] > max) {
          max = smoothed_wave[i + 1];
          maxIndex = i + 1;
        } else if(smoothed_wave[i + 1] < min) {
          min = smoothed_wave[i + 1];
        }
      }
      input[index].amplitude = 100*(max-min);
        #ifdef DEBUG4
      Serial.println(String::format("%d - Preliminary amplitude: %d", index, input[index].amplitude));
        #endif
    }
    #ifdef DEBUG4
    Serial.println("------------------");
    #endif
    return;
  }

void Sensors::bruteforceFrequencies(int measurementIndex/* = -1*/) {
    #ifdef DEBUG4
  Serial.println("------------------");
    #endif
  unsigned int index;
  unsigned int length;
  if(measurementIndex == -1) {
    index = 0;
    length = input_count;
  } else {
    index = measurementIndex;
    length = measurementIndex + 1;
  }
  for(; index < length; index++) {
    double error;
    double lowestError = -1;
    int xShift = 0;
    int period;
    int iterator;

    int sampleCap = measurement_samples;

    for(int periodRound = 0; periodRound < 3; periodRound++) {
      if(periodRound == 1) {
        sampleCap = period / measurementDuration;
      } else if(periodRound == 2) {
        sampleCap = measurement_samples;
      }
      bool foundPeriod = false;
      bool foundxShift = false;
      lowestError = -1;

      int periodMax = input[index].periodMax;
      int periodMin = input[index].periodMin;
      int xShiftMax = input[index].xShiftMax;
      int xShiftMin = input[index].xShiftMin;

      while(!foundPeriod || !foundxShift) {
                // Recalculate period
        if(index == 0 && periodRound != 1) {
          iterator = (periodMax-periodMin) / regression_n;
          if(iterator < 1) {
            foundPeriod = true;
            iterator = 1;
          }

          for(int i = periodMin; i < periodMax; i+= iterator) {
                        #ifdef DEBUG5
            Serial.println(String::format("%d - Trying period %d, xShift %d", index, i, xShift));
                        #endif
            error = 0;
            for(int j = 0; j < sampleCap; j++) {
              if(samples[index][j] > input[index].waveMin && samples[index][j] < input[index].waveMax) {
                error += waveError(index, j, xShift, input[index].amplitude, i);
              }
            }
            error = sqrt(error);
                        #ifdef DEBUG5
            Serial.println(String::format("%d - Error %f", index, error));
                        #endif
            if(error < lowestError || lowestError < 0) {
              lowestError = error;
              period = i;
            }
          }
          periodMin = period - iterator;
          periodMax = period + iterator;
                    #ifdef DEBUG5
          Serial.println(String::format("%d - Period - %d", index, period));
                    #endif
        } else if(index != 0) {
          period = input[0].period;
          foundPeriod = true;
        } else {
          foundPeriod = true;
        }
        if(periodRound != 2) {
                    // Recalculate xShift
          iterator = (xShiftMax-xShiftMin) / regression_n;
          if(iterator < 1) {
            foundxShift = true;
            iterator = 1;
          }
          for(int i = xShiftMin; i < xShiftMax; i+= iterator) {
                        #ifdef DEBUG5
            Serial.println(String::format("%d - Trying period %d, xShift %d", index, period, i));
                        #endif
            error = 0;
            for(int j = 0; j < sampleCap; j++) {
              if(samples[index][j] > input[index].waveMin && samples[index][j] < input[index].waveMax) {
                error += waveError(index, j, i, input[index].amplitude, period);
              }
            }
            error = sqrt(error);
                        // DEBUG
            if(error < 1) {
              Serial.println("Error was incorrect");
              Serial.println(String::format("xShift - %d | yShift - %d | amplitude - %d | period - %d", i, input[index].yShift, input[index].amplitude, period));
            }
                        #ifdef DEBUG5
            Serial.println(String::format("%d - Error %f", index, error));
                        #endif
            if(error < lowestError || lowestError < 0) {
              lowestError = error;
              xShift = i;
            }
          }
          xShiftMax = xShift + iterator;
          xShiftMin = xShift - iterator;
                    #ifdef DEBUG5
          Serial.println(String::format("%d - xShift - %d", index, xShift));
                    #endif
        } else {
          foundxShift = true;
        }
      }
      if(index != 0) {
        break;
      }
    }
    input[index].error = lowestError;
        #ifdef DEBUG4
        #ifdef DEBUG3
    Serial.println(String::format("1.%d xShift: %d", index, xShift));
        #endif
    Serial.println(String::format("1.%d period: %d", index, period));
    Serial.println(String::format("1.%d error: %f", index, lowestError));
    Serial.println("------------------");
        #endif
    input[index].xShift = xShift;
    input[index].period = period;

    if(input[index].period < 15002) {
      input[index].frequency = 0;
    } else {
      input[index].frequency = (((double)1000 * (double)1000. / (double)input[index].period));
      input[index].frequency = (pow((double)input[index].frequency, 3)*(double)input[index].fa + pow((double)input[index].frequency, 2)*(double)input[index].fb + (double)input[index].frequency*(double)input[index].fc + (double)input[index].fd);
    }
  }
  return;
}

void Sensors::bruteforceAmplitudes(int measurementIndex/* = -1*/) {
    #ifdef DEBUG4
Serial.println("------------------");
    #endif
unsigned int index;
unsigned int length;
if(measurementIndex == -1) {
  index = 0;
  length = input_count;
} else {
  index = measurementIndex;
  length = measurementIndex + 1;
}
measurementsValid = true;
for(; index < length; index++) {
  double error;
  double lowestError = -1;
  int iterator;
  int amplitude = 0;

  bool foundAmplitude = false;

  int amplitudeMin = input[index].amplitudeMin;
  int amplitudeMax = input[index].amplitudeMax;

  while(!foundAmplitude) {
            // Recalculate amplitude
    iterator = (amplitudeMax-amplitudeMin) / regression_n;
    if(iterator < 100) {
      foundAmplitude = true;
      iterator = 100;
    }
    for(int i = amplitudeMin; i < amplitudeMax; i+= iterator) {
                #ifdef DEBUG5
      Serial.println(String::format("%d - Trying amplitude %d", index, i));
                #endif
      error = 0;
      for(unsigned int j = 0; j < measurement_samples; j++) {
        if(samples[index][j] > input[index].waveMin && samples[index][j] < input[index].waveMax) {
          error += waveError(index, j, input[index].xShift, i, input[index].period);
        }
      }
      error = sqrt(error);
                #ifdef DEBUG5
      Serial.println(String::format("%d - Error %f", index, error));
                #endif
      if(error < lowestError || lowestError < 0) {
        lowestError = error;
        amplitude = i;
      }
    }
    amplitudeMin = amplitude - iterator;
    amplitudeMax = amplitude + iterator;
  }
  if(lowestError < input[index].error) {
    input[index].error = lowestError;
    input[index].amplitude = amplitude;
  }
  input[index].rms = (pow((double)input[index].amplitude, 3)*(double)input[index].a + pow((double)input[index].amplitude, 2)*(double)input[index].b + (double)input[index].amplitude*(double)input[index].c + (double)input[index].d);
  if(input[index].error > maxError) {
    measurementsValid = false;
            #ifndef DEBUG1
    return;
            #endif
  }
        #ifdef DEBUG4
  Serial.println(String::format("2.%d amplitude: %d", index, amplitude));
  Serial.println(String::format("2.%d error: %f", index, lowestError));
  Serial.println("------------------");
        #endif
}
return;
}

bool Sensors::checkStatus() {
  for(unsigned int i = 0; i < status_samples; i++) {
    if(samples[0][i] > inputActiveThreshold) {
      return true;
    }
  }
  return false;
}

void Sensors::setOutputs(int mode) {
  switch(mode) {
    case 0:
    for(int i = 0; i < 4; i++) {
      input[i].frequency = 0;
      input[i].rms = 0;
      input[i].error = 0;
    }
    inputActive = false;
    measurementsValid = true;
    break;
  }
}
#ifdef DEBUG1
void Sensors::printWaves(int mode/* = -1*/) {
switch(mode) {
  case -1:
  for(unsigned int j = 0; j < input_count; j++) {
    for(unsigned int i = 0; i < measurement_samples; i++) {
      Serial.println(String::format("%d, %f", i*measurementDuration, samples[j][i]));
    }
    for(unsigned int i = 0; i < measurement_samples; i++) {
      Serial.println(String::format("%d, %f", i*measurementDuration, simulateWave(input[j].yShift, input[j].rectified, input[j].xShift, input[j].xShiftMax, input[j].amplitude, i, input[j].period)));
    }
  }
  break;
  default:
  int j = mode;
  for(unsigned int i = 0; i < measurement_samples; i++) {
    Serial.println(String::format("%d, %f", i*measurementDuration, samples[j][i]));
  }
  for(unsigned int i = 0; i < measurement_samples; i++) {
    Serial.println(String::format("%d, %f", i*measurementDuration, simulateWave(input[j].yShift, input[j].rectified, input[j].xShift, input[j].xShiftMax, input[j].amplitude, i, input[j].period)));
  }
  break;
}
}
#endif
