package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Resources.*;

/**
 * This class provides other classes with methods relevant for navigation, such
 * as travelTo, turnTo and distance conversions.
 * 
 * @author Tony
 *
 */

public class Navigation {

  private static boolean traveling;
  private static double currentPosition[] = new double[3];


  /**
   * This method makes the robot travel to a specified point (x,y)
   * 
   * @param x tile coordinate
   * @param y tile coordinate
   */

  public void travelTo(double x, double y) {
    // get the current position of the robot
    boolean isAvoiding = false;
    currentPosition = odometer.getXYT();
    System.out.println("in travel method, currentPosition" + currentPosition[0] + " " + currentPosition[1] + " "
        + currentPosition[2]);
    // calculate the trajectory required to get to next way-point
    double deltaX = x * TILE_SIZE - currentPosition[0];
    double deltaY = y * TILE_SIZE - currentPosition[1];
    double nextTheta = (Math.atan2(deltaX, deltaY)) * 180 / Math.PI;
    double trajectory = Math.hypot(deltaX, deltaY);

    // travel to next way point
    traveling = true;
    turnTo(nextTheta); // turn in correct direction

    // move forward
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.rotate(convertDistance(trajectory), true);
    leftMotor.rotate(convertDistance(trajectory), false);


    leftMotor.stop(true);
    rightMotor.stop();
    traveling = false;

  } 

  //
  //	/**
  //	 * Travels in a straight line in order to avoid an obstacle, to be used after
  //	 * turning 90 degrees
  //	 * 
  //	 * @param distance (cm) to be traveled
  //	 */
  //	public void clearObstacle(double distance) {
  //		leftMotor.rotate(convertDistance(distance), true);
  //		rightMotor.rotate(convertDistance(distance), false);
  //		leftMotor.stop();
  //		rightMotor.stop();
  //	}

  /**
   * This method turns the robot to a specified angle
   * 
   * @param theta turn angle
   */

  public void turnTo(double theta) {
    // determine the correction in angle required
    currentPosition = odometer.getXYT();
    //System.out.println("in turnTo method, the wanted turnto theta: " + theta);
    //System.out.println("in turnTo method, currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " "
    //+ currentPosition[2]);
    double error = theta - currentPosition[2];
    //System.out.println("error: " + error);
    if (error < -180.0) { // if the error is less than -180 deg
      error = error + 360; // add 360 degrees to the error, for a minimum angle
      turnRight(error);
    } else if (error > 180.0) { // if the error is greater than 180 deg
      error = error - 360; // subtract 360 degrees to the error, for a minimum angle
      turnLeft(error);
    } else if (error < 0) {
      turnLeft(error);
    } else {
      turnRight(error);
    }
  }

  /**
   * This method turns the robot left by a given angle. It is used by the turnTo
   * method.
   * 
   * @param theta turn angle
   */

  public void turnLeft(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(-convertAngle(Math.abs(theta)), true);
    rightMotor.rotate(convertAngle(Math.abs(theta - 3)), false);
    //System.out.println("leftmotor turn left: " + (-convertAngle(Math.abs(theta))));
    //System.out.println("rightmotor turn left: " + convertAngle(Math.abs(theta)));
  }

  /**
   * This method turns the robot right by a given angle. It is used by the turnTo
   * method.
   * 
   * @param theta turn angle
   */

  public void turnRight(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(Math.abs(theta)), true);
    rightMotor.rotate(-convertAngle(Math.abs(theta)), false);

  }

  /**
   * This method converts a distance in cm to the number of wheel rotations needed
   * to cover that same distance.
   *
   * @param distance Distance in cm
   * @return Number of wheel rotations
   */
  public int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * This method converts a distance in degrees to the number of wheel rotations
   * needed to cover that same distance.
   *
   * @param angle angle in degrees
   * @return Number of wheel rotations
   */
  public int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }

  /**
   * This method checks if the robot is currently navigating
   * 
   * @return true if the robot is moving
   */

  static boolean isNavigating() {
    return traveling;
  }


}