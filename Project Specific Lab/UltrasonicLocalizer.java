package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Resources.*;

import lejos.hardware.Sound;



/**
 * Implements either falling or rising edge localization which uses the distance to the wall returned by an ultrasonic
 * sensor to orient the EV3 robot with a 0 degrees heading
 */
public class UltrasonicLocalizer implements Runnable {

  private static float[] usData = new float[usSensor.sampleSize()];

 
  /**
   * Wall distance
   */
  private double d = 30.00;

  /**
   * Gap distance
   */
  private double k = 3.0;
  private double fallingOffset = 15;
  private double deltaTheta;
 
  
  /**
   * begins either falling edge or rising edge localization
   * 
   * @param fallingEdge - if true: falling edge, false: rising edge
   */
  public void run() {
 
    
    // set the motor speeds
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    fallingEdge();
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /**
   * Orients the robot to have a 0 degrees heading using falling edge localization
   */
  private void fallingEdge() {
    
	  double theta1;
	    double theta2;
	    double turningAngle;
	    // Rotate until no wall detected
	    while (fetchData() < d + k) {
	      leftMotor.backward();
	      rightMotor.forward();
	    }
	    // Rotate left
	    while (fetchData() > d) {
	      leftMotor.backward();
	      rightMotor.forward();
	    }
	    Sound.beep();


	    theta1 = odometer.getXYT()[2];


	    // Rotate right until no wall detected
	    while (fetchData() < d + k) {
	      leftMotor.forward();
	      rightMotor.backward();
	    }

	    // Rotate right until walls detected
	    while (fetchData() > d) {
	      leftMotor.forward();
	      rightMotor.backward();
	    }
	    Sound.beep();

	    theta2 = odometer.getXYT()[2];

	    leftMotor.stop(true);
	    rightMotor.stop();

	    if (theta1 < theta2) {
	      deltaTheta = 15 - (theta1 + theta2) / 2;


	    } else if (theta1 > theta2) {
	      deltaTheta = 235 - (theta1 + theta2) / 2;
	    }

	    turningAngle = deltaTheta + odometer.getXYT()[2];
	    // Rotate robot to 0 degrees
	    leftMotor.rotate(-convertAngle(turningAngle - fallingOffset), true);
	    rightMotor.rotate(convertAngle(turningAngle - fallingOffset), false);

	    odometer.setXYT(0.0, 0.0, 0.0);
	  
	  
	  
	  
    // start LightLocalLizer.
    new Thread (new LightLocalizer()).start();
}


 

  /**
   * Converts a distance in centimeters to the corresponding wheel rotations required
   * 
   * @param distance (cm)
   * @return distance in wheel rotations
   */
  int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts a direction in degrees to the corresponding wheel rotations required
   * 
   * @param direction (deg)
   * @return distance in wheel rotations
   */
  int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }

  /**
   * fetch the distance between wall and robot from the sensor
   *
   * @return distance
   */
  private int fetchData() {
      usSensor.fetchSample(usData, 0);
      return (int) (usData[0] * 100);
  }
  


}
