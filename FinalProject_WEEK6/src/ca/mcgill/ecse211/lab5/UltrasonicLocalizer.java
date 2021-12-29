package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Resources.*;

import lejos.hardware.Sound;

/**
 * This class performs localization using the ultrasonic sensor to locate and
 * position the robot to an approximation of the north angle. It utilizes the
 * falling edge method
 * 
 * @author Tony Ou
 * @author Xinran Li
 *
 */

public class UltrasonicLocalizer {

  //private static float[] usData = new float[usSensor.sampleSize()];
  private double wall_distance = 30;
  private double margin = 3;
  private double deltaTheta;
  Navigation navigation = new Navigation();

  /**
   * This method runs the localization by setting the motor speeds and calling the
   * fallingEdge() method. It then sleeps for 1 second.
   * 
   */

  public void localize() {

    // set the motor speeds
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    fallingEdge();
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * This method performs falling edge localization. After detecting 2 angles from
   * an arbitrary distance from the wall, it calculates the angle needed to turn
   * north. After turning north, the Odometer values are set to 0.
   * 
   */

  private void fallingEdge() {

    double theta1;
    double theta2;

    theta1 = getAngleAFall();
    theta2 = getAngleBFall();

    // Calculate deltaT (degrees)

    if (theta1 < theta2) {
      deltaTheta = 45 - (theta1 + theta2) / 2.0;
    } else {
      deltaTheta = 225 - (theta1 + theta2) / 2.0;
    }

    double updatedAngle = odometer.getXYT()[2] + deltaTheta + 180;
    odometer.setXYT(0.0, 0.0, updatedAngle);

    // Turn to 0 degrees

    navigation.turnTo(0);

    // Update odometer values

    odometer.setXYT(0.0, 0.0, 0.0);
  }

  /**
   * Method to calculate the first angle of falling edge clock
   * 
   * @return odometer at the first falling edge
   */

  private double getAngleAFall() {

    // Turn right until open area is detected

    while (ultrasonicPoller.getDistance() < wall_distance + margin) {
      leftMotor.forward();
      rightMotor.backward();
    }

    // Keep turning until wall is detected

    while (ultrasonicPoller.getDistance() > wall_distance) {
      leftMotor.forward();
      rightMotor.backward();
    }

    // Alert with sound and stop motors

    //Sound.beep();
    try {
      Thread.sleep(300);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    leftMotor.stop(true);
    rightMotor.stop();

    // Record the first angle value (degrees)

    return odometer.getXYT()[2];
  }

  /**
   * Method to calculate the second angle of falling edge clock
   * 
   * @return odometer at the second falling edge
   */

  private double getAngleBFall() {

    // Turn right until open area is detected

    while (ultrasonicPoller.getDistance() < wall_distance + margin) {
      leftMotor.backward();
      rightMotor.forward();
    }

    // Keep turning until wall is detected

    while (ultrasonicPoller.getDistance() > wall_distance) {
      leftMotor.backward();
      rightMotor.forward();
    }

    // Alert with sound and stop motors

    try {
      Thread.sleep(300);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    //Sound.beep();
    leftMotor.stop(true);
    rightMotor.stop();

    // Record the second angle value (degrees)

    return odometer.getXYT()[2];

  }

}