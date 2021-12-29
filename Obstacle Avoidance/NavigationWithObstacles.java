package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class NavigationWithObstacles extends Thread {

  private static boolean navigating = true;
  
  private boolean obstacleDodging = false;

  int distance;

  private int filterControl = 0;

  private static SampleProvider myDistance = US_SENSOR.getMode("Distance");

  private static float[] usData = new float[myDistance.sampleSize()];

  /**
   * Run function, input waypoints here
   */

  public void run() {

    travelTo(1 * TILE_SIZE, 2 * TILE_SIZE );
    travelTo(2 * TILE_SIZE, 3 * TILE_SIZE );
    travelTo(2 * TILE_SIZE, 1 * TILE_SIZE );
    travelTo(3 * TILE_SIZE, 2 * TILE_SIZE );
    travelTo(3 * TILE_SIZE, 3 * TILE_SIZE );

  }

  /**
   * This method calculates the vehicle displacement through x/y
   * and the turning angle (theta). It then sets the rotations 
   * needed to reach the right waypoint. Also includes an algorithm to dodge
   * the obstacles
   * 
   * @param x
   * @param y
   */

  public void travelTo(double x, double y) {

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

    // Make the robot turn to the right direction

    turnTo(theta);

    // Cover the distance to the waypoint

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // Return two values to allow an additional thread for the US sensor

    leftMotor.rotate(distanceToRotations(displacement), true);
    rightMotor.rotate(distanceToRotations(displacement), true);

    // Sensor motor setup

    sensorMotor.resetTachoCount();
    sensorMotor.setAcceleration(1000);
    sensorMotor.setSpeed(SCAN_SPEED);

    //    System.out.println("Initial tacho count: " + sensorMotor.getTachoCount());

    // When the robot is moving, move the sensor

    while (leftMotor.isMoving() || rightMotor.isMoving()) {

      // Fetch distance from sensor and filter

      US_SENSOR.getDistanceMode().fetchSample(usData, 0); // acquire distance data in meters
      distance = (int) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
      filter(distance);

      // Turn left until sensor distance is within bandCenter

      if (distance <= BAND_CENTER) {
        leftMotor.stop(true);
        rightMotor.stop(false);
        navigating = false;
      }

      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }

      // Obstacle avoidance
      if (!this.isNavigating()) {
        Sound.beep(); // Alert using sound
        avoidObstacle();
        sensorMotor.rotateTo(0);
        navigating = true;
        travelTo(x, y);
        return;
      }

      // When vehicle reach a waypoint, reset sensor to rest position
      sensorMotor.rotateTo(0);

    }
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

    // Get the minimal angle first

    double turnAngle = getMinAngle(theta - Math.toRadians(odometer.getXYT()[2]));

    // Return one value immediately so only 3 threads are running

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

  /**
   * This method gets the distance read by the ultrasonic sensor
   * 
   * @return
   */

  public int readUSDistance() {
    return this.distance;
  }

  /**
   * This method uses the bang-bang controller to avoid obstacles
   */

  public void avoidObstacle() {
    
    obstacleDodging = true;
    sensorMotor.stop();

    // Turn 90degrees right

    turnTo(Math.toRadians(odometer.getXYT()[2]) - Math.PI / 2);

    // Adjust sensor motor to peak angle for obstacle detection

    sensorMotor.rotateTo(OBSTACLE_SENSOR_ANGLE);

    // Get the initial angle

    double endAngle = Math.toRadians(odometer.getXYT()[2]) + Math.PI * 0.8;

    // Bang-bang controller from lab 1 to avoid obstacle 

    while (Math.toRadians(odometer.getXYT()[2]) < endAngle) {
      US_SENSOR.fetchSample(usData, 0);
      distance = (int) (usData[0] * 100.0);
      int errorDistance = BAND_CENTER - distance;

      if (odometer.getXYT()[2] > 50.0 && odometer.getXYT()[2] < 60.0) {
        obstacleDodging = false;
        break;
      }

      // Drive straight
      if (Math.abs(errorDistance) <= BAND_WIDTH) {
        leftMotor.setSpeed(OBSTACLE_LOW);
        rightMotor.setSpeed(OBSTACLE_LOW);
        leftMotor.forward();
        rightMotor.forward();
      }

      // Robot is on the inside of the offset and over the bandwidth
      else if (errorDistance > 0) {

        rightMotor.setSpeed(OBSTACLE_LOW);
        leftMotor.setSpeed(OBSTACLE_TURN_OUT_SPEED);
        rightMotor.forward();
        leftMotor.backward();
      }

      // Robot is on the outside of the offset and over the bandwidth
      else if (errorDistance < 0) {

        rightMotor.setSpeed(OBSTACLE_LOW);
        leftMotor.setSpeed(OBSTACLE_TURN_IN_SPEED);
        rightMotor.forward();
        leftMotor.forward();
      }
    }

    // Finished cornering the obstacle

    leftMotor.stop();
    rightMotor.stop();

  }

  /**
   * This method filters out bad values (from lab1)
   * 
   * @param distance
   */

  private void filter(int distance) {
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the filter value
      filterControl++;
    } else if (distance >= 255) {
      // Repeated large values, so there is nothing there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave distance alone.
      filterControl = 0;
      this.distance = distance;
    }
  }
}
