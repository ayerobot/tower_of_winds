# tower_of_winds
Arduino code for an architectural model of Toyo Ito's Tower of Winds. Written for Brown University HIAA0860: Contemporary Architecture.

Performs a random walk on a sphere in RGB color space, basic peak detection, and correlates the variance of the random walk to a binned frequency spectrum.

Requires the ELM-ChaN FFT library. Weights for the FFT, as well as knowledge about how to do the system-level ADC code, came from https://learn.adafruit.com/piccolo/code. Possibly requires more tuning to work in louder environments. 
