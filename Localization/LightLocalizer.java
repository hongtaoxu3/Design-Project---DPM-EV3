package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer {

  // Initialize variables for the color sensor

  private static final Port colourSampler = LocalEV3.get().getPort("S2");
  private SensorModes colourSamplerSensor = new EV3ColorSensor(colourSampler);
  private SampleProvider colourSensorValue = colourSamplerSensor.getMode("Red");
  private float[] colourSensorValues = new float[colourSamplerSensor.sampleSize()];

  // Initialize the colorValue

  private float colorValue;

  // Initialize array to store line angles

  private double[] lineMeasures = new double[4];

  /**
   * This method localizes the robot using the light sensor
   * 
   */

  public void localize() {

    // Initializes required values
    boolean onBlackLine = false;
    double dx, dy, thetaX, thetaY;

    // Set default speed for motors and go to the origin (0,0)

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    goToOrigin();

    // Starting index for line detection

    int index = 0;

    // Keep running the loop until all lines are detected

    while (index < 4) {

      // Constantly turning right

      leftMotor.forward();
      rightMotor.backward();

      // Read the color sensor value

      colorValue = readColorData();

      // If a line is detected, update the array with its theta value

      if (colorValue < 0.3 && !onBlackLine) {
        lineMeasures[index] = odometer.getXYT()[2];
        Sound.beep();
        index++;
        onBlackLine = true;
      }
      else {
        onBlackLine = false;
      }
    }

    // Interrupt the motors

    leftMotor.stop(true);
    rightMotor.stop();

    // Calculate thetas from 0 using angles in array

    thetaX = lineMeasures[2] - lineMeasures[0];
    thetaY = lineMeasures[3] - lineMeasures[1];

    // Calculate distance from 0

    dx = -1 * COLOR_SENSOR_LENGTH * Math.cos(Math.toRadians(thetaY / 2));
    dy = -1 * COLOR_SENSOR_LENGTH * Math.cos(Math.toRadians(thetaX / 2));

    // Update odometer values and move to origin

    odometer.setXYT(dx, dy, odometer.getXYT()[2]);

    travelTo(0, 0);

    // Stop motors

    leftMotor.stop(true);
    rightMotor.stop();

    // Turn to 0 degrees

    double currentAngle = odometer.getXYT()[2];

    // Set speed and motor direction

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();

    // Keep turning until angle is pointing north

    while(true) {
      if (currentAngle >= 360 || currentAngle <= 10) {
        break;
      }
      else {
        currentAngle = odometer.getXYT()[2];
      }
    }

    // Stop motors

    leftMotor.stop(true);
    rightMotor.stop();

  }

  /**
   * This method makes the robot go to the origin
   * 
   */

  public void goToOrigin() {

    // Make the robot face backwards to the origin

    turnTo(225);

    colorValue = readColorData();

    // Move robot backwards until it detects a line

    while (colorValue > 0.3) {
      colorValue = readColorData();
      leftMotor.backward();
      rightMotor.backward();
    }

    // Interrupt motors

    Sound.beep();
    leftMotor.stop(true);
    rightMotor.stop();

    // Adjust light sensor position

    leftMotor.rotate(distanceToRotations(COLOR_SENSOR_LENGTH), true);
    rightMotor.rotate(distanceToRotations(COLOR_SENSOR_LENGTH), false);
    leftMotor.stop(true);
    rightMotor.stop(true);
  }

  /**
   * This method calculates the vehicle displacement through x/y
   * and the turning angle (theta). It then sets the rotations 
   * needed to reach the x/y point. (taken from lab3)
   * 
   * @param x
   * @param y
   */

  public void travelTo(double x, double y) {

    // Initializes variables

    double dx, dy, displacement, theta;

    // Get the current x/y position

    double currentX = odometer.getXYT()[0];
    double currentY = odometer.getXYT()[1];
    double currentTheta = Math.toRadians(odometer.getXYT()[2]);

    // Calculate x and y components from destination

    dx = x - currentX;
    dy = y - currentY;

    // Calculate displacement and turning angle needed

    displacement = Math.hypot(dx, dy);
    theta = Math.atan2(dx, dy) - currentTheta;

    // Turn to destination

    turnTo(theta);

    // Travel the calculated distance

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(distanceToRotations(displacement), true);
    rightMotor.rotate(distanceToRotations(displacement), false);

    leftMotor.stop(true);
    rightMotor.stop(false);
  }

  /**
   * This method makes the robot turn to theta (taken from lab3)
   * 
   * @param theta
   */

  public void turnTo(double theta) {

    // Get the minimum angle

    theta = getMinAngle(theta);

    // If angle is negative, turn left

    if (theta < 0) {
      leftMotor.rotate(-radToDeg(-Math.toRadians(theta)), true);
      rightMotor.rotate(radToDeg(-Math.toRadians(theta)), false);
    }

    // If angle is positive, turn right

    else {
      leftMotor.rotate(radToDeg(Math.toRadians(theta)), true);
      rightMotor.rotate(-radToDeg(Math.toRadians(theta)), false);
    }
  }

  /**
   * This method returns the current value of the color sensor
   * 
   * @return
   */

  private float readColorData() {
    colourSensorValue.fetchSample(colourSensorValues, 0);
    return colourSensorValues[0];
  }

  /**
   * This method takes a distance and converts it to the number of wheel
   * rotations needed (taken from lab3)
   * 
   * @param distance
   * @return
   */

  private int distanceToRotations(double distance) {
    return (int) (180.0 * distance / (Math.PI * WHEEL_RAD));
  }

  /**
   * This method converts the distance angle (taken from lab3)
   * 
   * @param angle
   * @return
   */

  private int radToDeg(double angle) {
    return distanceToRotations(TRACK * angle / 2);
  }

  /**
   * This method calculates the minimum value of an angle
   * (taken from lab3)
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

}
