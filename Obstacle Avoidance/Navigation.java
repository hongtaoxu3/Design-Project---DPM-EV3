package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;

/**
 * This class implements methods for the navigation of the robot through waypoints
 * @author Tony
 *
 */

public class Navigation extends Thread {

  // The robot is not moving initially

  private static boolean navigating = false;

  /**
   * Run function, input waypoints here
   */

  public void run() {
    travelTo(3 * TILE_SIZE, 2 * TILE_SIZE );
    travelTo(2 * TILE_SIZE, 2 * TILE_SIZE );
    travelTo(2 * TILE_SIZE, 3 * TILE_SIZE );
    travelTo(3 * TILE_SIZE, 1 * TILE_SIZE );
  }

  /**
   * This method calculates the vehicle displacement through x/y
   * and the turning angle (theta). It then sets the rotations 
   * needed to reach the right waypoint.
   * 
   * @param x
   * @param y
   */

  public void travelTo(double x, double y) {

    // Robot starts navigating

    navigating = true;

    // Initialize variables

    double dx, dy, displacement, theta;

    // Get current position

    double currentX = odometer.getXYT()[0];
    double currentY = odometer.getXYT()[1];

    // Get the displacement and the angle to reach the
    // waypoint at (x, y)

    dx = x - currentX;
    dy = y - currentY;
    displacement = Math.hypot(dx, dy);
    theta = Math.atan2(dx, dy);

    // Turn to theta

    turnTo(theta);

    // Cover the distance to the waypoint

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // Return one value immediately so only 3 threads are running

    leftMotor.rotate(distanceToRotations(displacement), true);
    rightMotor.rotate(distanceToRotations(displacement), false);

    // Stops the robot and set navigating to false

    leftMotor.stop();
    rightMotor.stop();
    navigating = false;
  }

  /**
   * This method takes a distance and converts it to the
   * number of wheel rotations needed
   * 
   * @param distance
   * @return
   */

  private int distanceToRotations(double distance) {
    return (int) (180.0 * distance / (Math.PI * WHEEL_RAD));
  }

  /**
   * This method makes the robot turn to the right waypoint
   * 
   * @param theta
   */

  public void turnTo(double theta) {

    // Set rotate speed

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // Get the minimal turn angle

    double turnAngle = getMinAngle(theta - Math.toRadians(odometer.getXYT()[2]));

    // Return one value immediately so only 3 threads are running
    // Rotate each motor to the right direction

    leftMotor.rotate(radToDeg(turnAngle), true);
    rightMotor.rotate(-radToDeg(turnAngle), false);
  }

  /**
   * This method takes an angle and uses a formula
   * to convert it to its minimum value
   * 
   * @param angle
   * @return
   */

  public double getMinAngle(double angle) {
    if (angle > Math.PI) {
      angle -= 2 * Math.PI;
    } else if (angle < -Math.PI) {
      angle += 2 * Math.PI ;
    }
    return angle;
  }

  /**
   * This method transforms the angle in rad to degrees
   * 
   * @param angle
   * @return
   */

  private int radToDeg(double angle) {
    return distanceToRotations(TRACK * angle / 2);
  }

  /**
   * This method returns if the robot is travelling or not
   * 
   * @return
   */

  public boolean isNavigating() {
    return navigating;
  }

}
