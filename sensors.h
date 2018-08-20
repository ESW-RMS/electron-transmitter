/*

  Stanford Engineers for a Sustainable World
  Remote Monitoring System | August 2018
  Brian Tanabe | btanabe@stanford.edu

  File: sensors.h
  --------------------------
  Code for recording voltage, frequency, 3-phase current, and power on the 2018 sensorboard using a Particle Electron

*/

#ifndef SENSORS_H
#define SENSORS_H

#define DEBUG1 // Verbose
//#define DEBUG2 // Prints final output
#define DEBUG3 // Generates random sample data
#define DEBUG4 // Super verbose
//#define DEBUG5 // Prints regression

class Sensors {
public:
/**********************************  SETUP  ***********************************/
  Sensors ();

/********************************  FUNCTIONS  *********************************/
  void		init();
  void    refreshStatus();
  void    refreshAll();
  void    fieldTest();
  bool    getGeneratorStatus();
  unsigned short    getVoltage();
  unsigned short    getFrequency();
  unsigned short    getCurrent_1();
  unsigned short    getCurrent_2();
  unsigned short    getCurrent_3();
  

private:
/*********************************  HELPERS  **********************************/

  double 	simulateWave(int yShift, bool rectified, int xShift, int xShiftMax, int amplitude, int iterator, int period);
	double 	waveError(int measurementIndex, int iterator, int xShift, int amplitude, int period);
	void	 	recordSamples();
	void 		analyzeSmoothedWaves(int measurementIndex = -1);
	void 		bruteforceFrequencies(int measurementIndex = -1);
	void 		bruteforceAmplitudes(int measurementIndex = -1);
	bool 		checkStatus();
	void 		setOutputs(int mode);
	#ifdef DEBUG1
	void 		printWaves(int mode = -1); // -1 --> all, 0-3 --> specific wave, uses a switch for easy customization
	#endif

/*********************************  OBJECTS  **********************************/

  struct Measurement {
  	int 					pin;
  	double 				rms;
  	double 				frequency;
  	double 				a; // rms cubic coefficient
  	double 				b; // rms quad coefficient
  	double 				c; // rms linear coefficient
  	double 				d; // rms constant
  	double 				fa; // freq cubic coefficient
  	double 				fb; // freq quad coefficient
  	double 				fc; // freq linear coefficient
  	double 				fd; // freq constant

  	unsigned int 	periodMin;
  	unsigned int 	period;
  	unsigned int 	periodMax;

  	unsigned int 	xShiftMin;
  	unsigned int 	xShift;
  	unsigned int 	xShiftMax;

  	unsigned int 	amplitudeMin;
  	unsigned int 	amplitude;
  	unsigned int 	amplitudeMax;

  	int 					yShift;
  	int 					waveMin;
  	int 					waveMax;
  	double 				error;
    bool          rectified;
};

	unsigned short 	voltage;
	unsigned short 	frequency;
	unsigned short 	current_1;
	unsigned short 	current_2;
	unsigned short 	current_3;

	static constexpr double 	pi = 3.1415926535; // pi
	static const unsigned int measurement_samples = 2000; // Number of samples to take .73 seconds worth of data
	static const unsigned int status_samples = 600; // Quick check for status - takes ~.2 seconds
	static const unsigned int input_count = 4;
	double samples[input_count][measurement_samples]; // Must be global to work on Particle (sampling array)
	static const int maxError = 500;
	static const int maxMeasurementAttempts = 10;
	static const int invalidPlaceholder = 9999;
	static constexpr double compressionMultiplier = 100;
	static const unsigned int inputActiveThreshold = 200;
	bool inputActive = false;
	static const unsigned int smoothing_n = 5; // Voltage wave mean smoothing bucket size
	double smoothed_wave[measurement_samples - smoothing_n + 1]; // Must be global to work on particle (smoothed voltage array)
	static const unsigned int regression_n = 10; // Feature matching stride
	int measurementDuration;
	bool measurementsValid;
	Measurement input[input_count];
};

#endif