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
  input[0].yShift = -330;
  input[0].waveMin = 800;
  input[0].waveMax = 4096;
  input[0].fa = 0; //
  input[0].fb = 1;
  input[0].fc = 0;
  input[0].a = 0; //
  input[0].b = 1;
  input[0].c = 0;
  input[0].rectified = true;
  input[0].ignore = false;

  // Current 1
  input[1].pin = A1;
  input[1].yShift = 1975;
  input[1].waveMin = -1;
  input[1].waveMax = 4096;
  input[1].fa = 0;
  input[1].fb = 1;
  input[1].fc = 0;
  input[1].a = 0;
  input[1].b = 1;
  input[1].c = 0;
  input[1].rectified = false;
  input[1].ignore = true;

  // Current 2
  input[2].pin = A2;
  input[2].yShift = 1975;
  input[2].waveMin = -1;
  input[2].waveMax = 4096;
  input[2].fa = 0;
  input[2].fb = 1;
  input[2].fc = 0;
  input[2].a = 0;
  input[2].b = 1;
  input[2].c = 0;
  input[2].rectified = false;
  input[2].ignore = true;

  // Current 3
  input[3].pin = A3;
  input[3].yShift = 1975;
  input[3].waveMin = -1;
  input[3].waveMax = 4096;
  input[3].fa = 0;
  input[3].fb = 1;
  input[3].fc = 0;
  input[3].a = 0;
  input[3].b = 1;
  input[3].c = 0;
  input[3].rectified = false;
  input[3].ignore = true;

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
            Serial.println("------------------");
            Serial.println(String::format("MEASUREMENT ATTEMPT %d", attempts+1));
        #endif

        recordSamples();
        
        if(checkStatus()) {
            analyzeSmoothedWaves();
            bruteforceFrequencies();
            bruteforceAmplitudes();
        } else {
            zeroMeasurements(); // Set currents to 0 if generator is off
        }

        if(measurementsValid) {
            break;
        }
    }

    calculatePower();

    
    #ifdef DEBUG2
        Serial.println("---------FINAL ERROR---------");
        Serial.println(String::format("Voltage Error - %f", input[0].error));
        Serial.println(String::format("Current 1 Error - %f", input[0].error));
        Serial.println(String::format("Current 2 Error - %f", input[0].error));
        Serial.println(String::format("Current 3 Error - %f", input[0].error));
        Serial.println("---------FINAL VALUES---------");
    #endif

    if(measurementsValid) {
        voltage = input[0].rms*compressionMultiplier;
        frequency = input[0].frequency*compressionMultiplier;
        current_1 = input[1].rms*compressionMultiplier;
        current_2 = input[2].rms*compressionMultiplier;
        current_3 = input[3].rms*compressionMultiplier;
        power = power*compressionMultiplier;

        #ifdef DEBUG1
            Serial.println("\n---------FINAL OUTPUT---------");
            Serial.println(String::format("Voltage RMS: %f", (double)voltage/(double)compressionMultiplier));
            Serial.println(String::format("Frequency: %f", (double)frequency/(double)compressionMultiplier));
            Serial.println(String::format("Current 1 RMS: %f", (double)current_1/(double)compressionMultiplier));
            Serial.println(String::format("Current 2 RMS: %f", (double)current_2/(double)compressionMultiplier));
            Serial.println(String::format("Current 3 RMS: %f", (double)current_3/(double)compressionMultiplier));
            Serial.println(String::format("Power: %f", (double)power/(double)compressionMultiplier));
            Serial.println("------------------\n");
        #endif

    } else {
        voltage = invalidPlaceholder;
        frequency = invalidPlaceholder;
        current_1 = invalidPlaceholder;
        current_2 = invalidPlaceholder;
        current_3 = invalidPlaceholder;
        power = invalidPlaceholder;
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

  bool Sensors::generatorIsOn() {
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

  unsigned short Sensors::getPower() {
    return (unsigned short)power;
  }

  double Sensors::waveError(int measurementIndex, int iterator, int xShift, int amplitude, int period) {
    return pow((samples[measurementIndex][iterator] - simulateWave(input[measurementIndex].yShift, input[measurementIndex].rectified, xShift, amplitude, iterator, period)), 2);
  }

  double Sensors::simulateWave(int yShift, bool rectified, int xShift, int amplitude, int iterator, int period) {
    if(rectified) {
      return ((double)yShift + ((double)amplitude/(double)100) * fabs(cos( 2.0 * pi *  (((double)xShift/(double)xShiftRangeMax) + (((double)iterator*measurementDuration) / (double)period)))));
    } else {
      return ((double)yShift + ((double)amplitude/(double)100) * cos( 2.0 * pi *  (((double)xShift/(double)xShiftRangeMax) + (((double)iterator*measurementDuration) / (double)period))));
    }
  }

  void Sensors::recordSamples() {
// Record samples and sampling time
#ifndef DEBUG4
    int sampleTime = -micros();
    for(unsigned int i = 0; i < measurement_samples; i++) {
      for(unsigned int j = 0; j < input_count; j++) {
        samples[j][i] = analogRead(input[j].pin);
      }
    }
    sampleTime += micros();
    measurementDuration = ((double)sampleTime / (double)measurement_samples);
#else
    measurementDuration = 160;
    srand(micros() % 1000000);
    for(int a = 0; a < 2; a++) { // The first iteration is to calibrate measurementDuration
        int p = 18000 + rand() % 4000; // Random period 18000-22000
        if(a > 0) {
            Serial.println("------------------");
            Serial.println("Generated Samples");
            Serial.println("------------------");
            Serial.println(String::format("Period: %d", p));
        }
        for(unsigned int i = 0; i < input_count; i++) {
          input[i].xShift = rand() % (periodRangeMax / 2);
          input[i].amplitude = 20 + rand() % amplitudeRangeMax;
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
            samples[j][i] = simulateWave(input[j].yShift, input[j].rectified, input[j].xShift, input[j].amplitude, i, p);
          }
        }
        if(a < 1) {
          sampleTime += micros();
          measurementDuration = ((double)sampleTime / (double)measurement_samples);
        }
    }

#endif
  #ifdef DEBUG1
            Serial.println("Samples recorded");
        #endif
}

void Sensors::analyzeSmoothedWaves() {
    #ifdef DEBUG2
    Serial.println("------------------");
    #endif
    for(unsigned int index = 0; index < input_count; index++) {
        if(!input[index].ignore) {
            double avg = 0;
            for(unsigned int i = 0; i < smoothing_n; i++) {
                avg += samples[index][i];
            }
            smoothed_wave[0] = avg/smoothing_n;
            double max = smoothed_wave[0];
            double min = smoothed_wave[0];
            for(int i = 0, len = measurement_samples-smoothing_n; i < len; i++) {
                avg -= samples[index][i];
                avg += samples[index][i+smoothing_n];
                smoothed_wave[i + 1] = avg/smoothing_n;
                if(smoothed_wave[i + 1] > max) {
                    max = smoothed_wave[i + 1];
                } else if(smoothed_wave[i + 1] < min) {
                    min = smoothed_wave[i + 1];
                }
            }
            input[index].amplitude = input[index].rectified ? 100*(max-min) : 50*(max-min);
            #ifdef DEBUG2
                Serial.println(String::format("%d - Preliminary amplitude: %d", index, input[index].amplitude));
            #endif
        }
    }
    #ifdef DEBUG1
        Serial.println("------------------");
        Serial.println("Preliminary analysis complete");
    #endif
    
     return;
}

void Sensors::bruteforceFrequencies() {
    #ifdef DEBUG2
        Serial.println("------------------");
    #endif
    
    for(unsigned int index = 0; index < input_count; index++) {
        if(!input[index].ignore) {
            double error;
            double lowestError = -1;
            int xShift = 0;
            int period = 0;
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

                int periodMax = periodRangeMax;
                int periodMin = periodRangeMin;
                int xShiftMax = xShiftRangeMax;
                int xShiftMin = xShiftRangeMin;

                while(!foundPeriod || !foundxShift) {
                // Recalculate period
                    if(index == 0 && periodRound != 1) {
                        iterator = (periodMax-periodMin) / regression_n;
                        if(iterator < 1) {
                            foundPeriod = true;
                            iterator = 1;
                        }

                        for(int i = periodMin; i < periodMax; i+= iterator) {
                            #ifdef DEBUG3
                                Serial.println(String::format("%d - Trying period %d, xShift %d", index, i, xShift));
                            #endif
                            error = 0;
                            for(int j = 0; j < sampleCap; j++) {
                                if(samples[index][j] > input[index].waveMin && samples[index][j] < input[index].waveMax) {
                                    error += waveError(index, j, xShift, input[index].amplitude, i);
                                }
                            }
                            error = sqrt(error);
                            #ifdef DEBUG3
                                Serial.println(String::format("%d - Error %f", index, error));
                            #endif
                            if(error < lowestError || lowestError < 0) {
                                lowestError = error;
                                period = i;
                            }
                        }
                        periodMin = period - iterator;
                        periodMax = period + iterator;
                        #ifdef DEBUG3
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
                            #ifdef DEBUG3
                                Serial.println(String::format("%d - Trying period %d, xShift %d", index, period, i));
                            #endif
                            error = 0;
                            for(int j = 0; j < sampleCap; j++) {
                                if(samples[index][j] > input[index].waveMin && samples[index][j] < input[index].waveMax) {
                                    error += waveError(index, j, i, input[index].amplitude, period);
                                }
                            }
                            error = sqrt(error);
                            #ifdef DEBUG3
                                Serial.println(String::format("%d - Error %f", index, error));
                            #endif
                            if(error < lowestError || lowestError < 0) {
                                lowestError = error;
                                xShift = i;
                            }
                        }
                        xShiftMax = xShift + iterator;
                        xShiftMin = xShift - iterator;
                        #ifdef DEBUG3
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
            #ifdef DEBUG2
                Serial.println(String::format("1.%d xShift: %d", index, xShift));
                Serial.println(String::format("1.%d period: %d", index, period));
                Serial.println(String::format("1.%d error: %f", index, input[index].error));
            #endif
            input[index].xShift = xShift;
            input[index].period = period;

            if(input[index].period < 15002) {
                input[index].frequency = 0;
            } else {
                input[index].frequency = (((double)1000 * (double)1000. / (double)input[index].period));
                input[index].frequency = evaluatePolynomial(input[index].fa, input[index].fb, input[index].fc, (double)input[index].frequency);
            }
        } else {
            input[index].xShift = invalidPlaceholder;
            input[index].period = invalidPlaceholder;
            input[index].frequency = invalidPlaceholder;
        }
    }
    #ifdef DEBUG1
        Serial.println("------------------");
        Serial.println("Frequency analysis complete");
    #endif
    
    return;
}

void Sensors::bruteforceAmplitudes() {
    #ifdef DEBUG2
        Serial.println("------------------");
    #endif

    measurementsValid = true;

    for(unsigned int index = 0; index < input_count; index++) {
        if(!input[index].ignore) {
            double error;
            double lowestError = -1;
            int iterator;
            int amplitude = 0;

            bool foundAmplitude = false;

            int amplitudeMin = amplitudeRangeMin;
            int amplitudeMax = amplitudeRangeMax;

            while(!foundAmplitude) {
                // Recalculate amplitude
                iterator = (amplitudeMax-amplitudeMin) / regression_n;
                if(iterator < 100) {
                  foundAmplitude = true;
                  iterator = 100;
                }
                for(int i = amplitudeMin; i < amplitudeMax; i+= iterator) {
                    #ifdef DEBUG3
                        Serial.println(String::format("%d - Trying amplitude %d", index, i));
                    #endif
                    error = 0;
                    for(unsigned int j = 0; j < measurement_samples; j++) {
                        if(samples[index][j] > input[index].waveMin && samples[index][j] < input[index].waveMax) {
                            error += waveError(index, j, input[index].xShift, i, input[index].period);
                        }
                    }
                    error = sqrt(error);
                    #ifdef DEBUG3
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
            input[index].rms = evaluatePolynomial(input[index].a, input[index].b, input[index].c, (double)input[index].amplitude);

            if(input[index].error > maxError) {
                measurementsValid = false;
                #ifdef DEBUG2
                    Serial.println(String::format("2.%d amplitude: %d", index, input[index].amplitude));
                    Serial.println(String::format("2.%d error: %f", index, input[index].error));
                #endif
                #ifdef DEBUG1
                    Serial.println("------------------");
                    Serial.println("ATTEMPT FAILED");
                    printWaves(0, false);
                    delay(200000);
                #endif
            return;
            }
            #ifdef DEBUG2
                Serial.println(String::format("2.%d amplitude: %d", index, input[index].amplitude));
                Serial.println(String::format("2.%d error: %f", index, input[index].error));
            #endif
        } else {
            input[index].amplitude = invalidPlaceholder;
            input[index].error = 0;
        }
    }
    #ifdef DEBUG1
        Serial.println("------------------");
        Serial.println("Wave analysis complete");
    #endif

    return;
}

bool Sensors::checkStatus() {
  for(unsigned int i = 0; i < status_samples; i++) {
    if((int)samples[0][i] > inputActiveThreshold+input[0].yShift) {
      return true;
    }
  }
  return false;
}

void Sensors::calculatePower() {
    #ifdef DEBUG2
        Serial.println("------------------");
    #endif
    int currentxShift;
    int voltagexShift;
    int linePower; 
    power = 0;
    for(int j = 1; j < 4; j++) {
        if(!input[j].ignore) {
            currentxShift = input[j].xShift % (xShiftRangeMax/4);
            voltagexShift = input[0].xShift % (xShiftRangeMax/4);

            if(currentxShift - voltagexShift > -20 && currentxShift - voltagexShift < 0) {
                currentxShift = voltagexShift;
            } else if(currentxShift - voltagexShift < -20) {
                currentxShift += xShiftRangeMax/4;
            }
            linePower = (double)input[0].rms * (double)input[1].rms * cos(2*pi*(currentxShift - voltagexShift)/xShiftRangeMax);
            power += linePower;
            Serial.println(String::format("Line Power %d: %f", j, linePower));
        } else {
            power = invalidPlaceholder;
            break;
        }
    }
    Serial.println(String::format("Total Power: %f", power));
    #ifdef DEBUG1
        Serial.println("------------------");
        Serial.println("Power calculations complete");
    #endif
}

void Sensors::zeroMeasurements() {
    for(int i = 0; i < 4; i++) {
        if(!input[i].ignore) {
            input[i].frequency = 0;
            input[i].rms = 0;
            input[i].amplitude = 0;
            input[i].error = 0;
        } else {
            input[i].frequency = invalidPlaceholder;
            input[i].rms = invalidPlaceholder;
            input[i].amplitude = invalidPlaceholder;
            input[i].error = 0;
        }
    }
    inputActive = false;
    measurementsValid = true;
    power = 0;
}

void Sensors::printWaves(int index, bool simulated) {
    for(unsigned int i = 0; i < measurement_samples; i++) {
        Serial.println(String::format("%d, %f", i*(int)measurementDuration, samples[index][i]));
    }
    if(simulated) {
        for(unsigned int i = 0; i < measurement_samples; i++) {
            Serial.println(String::format("%d, %f", i*(int)measurementDuration, simulateWave(input[index].yShift, input[index].rectified, input[index].xShift, input[index].amplitude, i, input[index].period)));
        }
    }
}

double Sensors::evaluatePolynomial(double a, double b, double c, double x) {
    return pow(x, 2)*a + x*b + c;
}