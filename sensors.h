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
#define DEBUG2 // Prints calculations - DEBUG1 should be enabled
//#define DEBUG3 // Prints regression - DEBUG1&2 should be enabled
//#define DEBUG4 // Generates random sample data

class Sensors {
public:
/**********************************  SETUP  ***********************************/
  Sensors ();

/********************************  FUNCTIONS  *********************************/
  void		init();
  void    refreshStatus();
  void    refreshAll();
  void    fieldTest();
  bool    generatorIsOn();
  unsigned short    getVoltage();
  unsigned short    getFrequency();
  unsigned short    getCurrent_1();
  unsigned short    getCurrent_2();
  unsigned short    getCurrent_3();
  unsigned short    getPower();
  

private:
/*********************************  HELPERS  **********************************/

  double 	simulateWave(int yShift, bool rectified, int xShift, int amplitude, int iterator, int period);
	double 	waveError(int measurementIndex, int iterator, int xShift, int amplitude, int period);
	void	 	recordSamples();
	void 		analyzeSmoothedWaves();
	void 		bruteforceFrequencies();
	void 		bruteforceAmplitudes();
  void    calculatePower();
	bool 		checkStatus();
	void 		setOutputs(int mode);
  double    evaluatePolynomial(double a, double b, double c, double x);
	void 		printWaves(int index, bool simulated); // -1 --> all, 0-3 --> specific wave, uses a switch for easy customization


/*********************************  OBJECTS  **********************************/

  struct Measurement {
  	int 					pin;
  	double 				rms;
  	double 				frequency;
  	double 				a;
  	double 				b;
  	double 				c;
  	double 				fa;
    double 				fb;
    double 				fc;

  	unsigned int 	period;
  	unsigned int 	xShift;
  	unsigned int 	amplitude;

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
  unsigned short totalPower;

  double  power;

	static constexpr double 	pi = 3.1415926535; // pi
	static const unsigned int measurement_samples = 2000; // Number of samples to take .73 seconds worth of data
	static const unsigned int status_samples = 600; // Quick check for status - takes ~.2 seconds
	static const unsigned int input_count = 4;
	double samples[input_count][measurement_samples]; // Must be global to work on Particle (sampling array)
	static const int maxError = 500;
	static const int maxMeasurementAttempts = 10;
	static const int invalidPlaceholder = 9999;
	static constexpr double compressionMultiplier = 100;
	static const int inputActiveThreshold = 100;
	bool inputActive = false;
	static const unsigned int smoothing_n = 5; // Voltage wave mean smoothing bucket size
	double smoothed_wave[measurement_samples - smoothing_n + 1]; // Must be global to work on particle (smoothed voltage array)
	static const unsigned int regression_n = 10; // Feature matching stride


  const unsigned int periodRangeMin = 15000;
  const unsigned int periodRangeMax = 25000;
  const unsigned int xShiftRangeMin = 0;
  const unsigned int xShiftRangeMax = 10000;
  const unsigned int amplitudeRangeMin = 1;
  const unsigned int amplitudeRangeMax = 409500;


	double measurementDuration;
	bool measurementsValid;
	Measurement input[input_count];
};

#endif