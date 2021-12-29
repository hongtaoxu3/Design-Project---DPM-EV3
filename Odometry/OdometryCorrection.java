package ca.mcgill.ecse211.lab2;

import static ca.mcgill.ecse211.lab2.Resources.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  
  // Initialize the color sensor

  private static final Port colourSampler = LocalEV3.get().getPort("S2");
  private SensorModes colourSamplerSensor = new EV3ColorSensor(colourSampler);
  private SampleProvider colourSensorValue = colourSamplerSensor.getMode("Red");
  private float[] colourSensorValues = new float[colourSamplerSensor.sampleSize()];
  private float lastValue = 0;
  
  // Initialize the odometer

  private Odometer odometer;
  
  // Initialize distance correction values

  private int counterStartY = 0;
  private double distanceCorrectionY = 0;
  
  // Initialize x and y counters for tiles

  private int counterX = 1;
  private int counterY = 1;
  
  // Positional variables

  private double theta;
  private double initialPosition[];

  /*
   * Here is where the odometer correction code should be run.
   */
  
  public void run() {
    
    long correctionStart, correctionEnd;
    
    // Create a new instance of an odometer each time the code is ran
    
    this.odometer = Odometer.getOdometer();

    while (true) {
      
      correctionStart = System.currentTimeMillis();
      
      // Get current value of the color sensor

      colourSensorValue.fetchSample(colourSensorValues, 0);
      float value = (colourSensorValues[0] * 1000);
      
      // Calculate the difference between the current and last color value
      
      float difference = value - lastValue;
      lastValue = value;
      
      // If there is a large difference, it probably encountered a black line

      if (difference < -40) {

        Sound.beep(); // Alert using sound

        initialPosition = odometer.getXYT(); // Get current direction (in degrees)
        theta = initialPosition[2] * 180 / Math.PI;

        // If the robot is pointing north

        if ((theta >= 315 && theta <= 360) || (theta >= 0 && theta <= 45)) {
          
          // Calculate the distance correction (Starting position to position of first beep)
          
          if (counterStartY == 0) {
            distanceCorrectionY = initialPosition[1];
            counterStartY++; // Increment so it does not calculate the correction again
          }
          
          // Update the y position with additional TILE_SIZE each time it encounters a line
          
          else {
            odometer.setY(distanceCorrectionY + counterY * TILE_SIZE);
            counterY++; // Increment the y counter
          }
        }

        // If the robot is pointing east
        
        // Update the x position with additional TILE_SIZE each time it encounters a line

        else if (theta > 45 && theta <= 135) {
          odometer.setX(counterX * TILE_SIZE);
          counterX++;
        }

        // If the robot is pointing south

        // Update the y position by decrementing the y counter first (opposite y direction)
        
        else if (theta > 135 && theta <= 225) {
          counterY--;
          odometer.setY(TILE_SIZE * counterY + distanceCorrectionY);
        }

        // If the robot is pointing west
        
        // Update the x position by decrementing the x counter first (opposite x direction)

        else if (theta > 225 && theta < 315) {
          counterX--;
          odometer.setX(TILE_SIZE * counterX);
        }
      }

      // this ensures the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }
  }

}
