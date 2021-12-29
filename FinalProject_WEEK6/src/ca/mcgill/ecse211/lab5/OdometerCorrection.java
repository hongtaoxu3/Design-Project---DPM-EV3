package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Resources.*;

/**
 * This class contains methods to correct the odometer readings, more
 * specifically the theta error.
 * 
 * @author Tony & Arianit
 *
 */

public class OdometerCorrection {

  Navigation navigation;
  Odometer odometer;

  /**
   * The default constructor for odometer correction
   * 
   * @param navigation A navigation object
   * @param odometer   An odometer object
   */

  public OdometerCorrection(Navigation navigation, Odometer odometer) {
    this.navigation = navigation;
    this.odometer = odometer;
  }

  /**
   * This method corrects the error between the odometer theta and the actual
   * orientation of the robot. It does so by recording the odometer coordinates
   * once a color sensor reads a line. When the other color sensor reads that same
   * line, we use trigonometry to get the actual orientation of the robot. We then
   * set the odometer to that angle and turn the robot to 0 degrees.
   * 
   */
  public void CorrectAngle() {

    leftMotor.setSpeed(180);
    rightMotor.setSpeed(180);
    leftMotor.forward();
    rightMotor.forward();

    boolean rightLineDetected = false;
    boolean leftLineDetected = false;

    float lastRightColorValue = 0;
    float lastLeftColorValue = 0;

    double[] leftSensorCoordinates = new double[3];
    double[] rightSensorCoordinates = new double[3];

    while (true) {

      float currentRightColorValue = fetchRightLightSample();
      float rightDifference = currentRightColorValue - lastRightColorValue;
      lastRightColorValue = currentRightColorValue;

      float currentLeftColorValue = fetchLeftLightSample();
      float leftDifference = currentLeftColorValue - lastLeftColorValue;
      lastLeftColorValue = currentLeftColorValue;

      if (rightDifference < -20) {
        rightLineDetected = true;
        rightSensorCoordinates = odometer.getXYT();
      }

      if (leftDifference < -20) {
        leftLineDetected = true;
        leftSensorCoordinates = odometer.getXYT();
      }

      if (rightLineDetected && leftLineDetected) {
        break;
      }
    }

    leftMotor.stop(true);
    rightMotor.stop();

    //double angleToCorrect = 0;
    //double degreesAngleToCorrect =0;

    System.out.println(
        "currentPosition: " + odometer.getXYT()[0] + " " + odometer.getXYT()[1] + " " + odometer.getXYT()[2]);


    //		
    //		
    //		if (rightSensorCoordinates[1] > leftSensorCoordinates[1]) {
    //			angleToCorrect = Math.asin((rightSensorCoordinates[1] - leftSensorCoordinates[1]) / TRACK);
    //			degreesAngleToCorrect = -Math.toDegrees(angleToCorrect);
    //			System.out.println("degreeangletocorrect*******: " + degreesAngleToCorrect);
    //		} else if (rightSensorCoordinates[1] < leftSensorCoordinates[1]) {
    //			angleToCorrect = Math.asin((leftSensorCoordinates[1] - rightSensorCoordinates[1]) / TRACK);
    //			degreesAngleToCorrect = -Math.toDegrees(angleToCorrect);
    //			System.out.println("degreeangletocorrect*******: " + degreesAngleToCorrect);
    //		}
    //		
    //		if (rightSensorCoordinates[0] > leftSensorCoordinates[0]) {
    //			angleToCorrect = Math.asin((rightSensorCoordinates[0] - leftSensorCoordinates[0]) / TRACK);
    //			degreesAngleToCorrect = -Math.toDegrees(angleToCorrect);
    //			System.out.println("degreeangletocorrect*******: " + degreesAngleToCorrect);
    //		} else if (rightSensorCoordinates[0] < leftSensorCoordinates[0]) {
    //			angleToCorrect = Math.asin((leftSensorCoordinates[0] - rightSensorCoordinates[0]) / TRACK);
    //			degreesAngleToCorrect = -Math.toDegrees(angleToCorrect);
    //			System.out.println("degreeangletocorrect*******: " + degreesAngleToCorrect);
    //		}


    double angleToCorrect = Math.asin((rightSensorCoordinates[1] - leftSensorCoordinates[1])/TRACK);
    double degreesAngleToCorrect =  Math.toDegrees(angleToCorrect);

    odometer.setXYT(odometer.getXYT()[0], odometer.getXYT()[1], odometer.getXYT()[2] + degreesAngleToCorrect);		


    System.out.println("Angle of Robot: " + degreesAngleToCorrect + "  "+ angleToCorrect+"  " + odometer.getXYT()[2]);
    System.out.println("Y difference: " + (rightSensorCoordinates[1] - leftSensorCoordinates[1]));

    if ((odometer.getXYT()[2] < 45 && odometer.getXYT()[2] > 0)
        || (odometer.getXYT()[2] > 315 && odometer.getXYT()[2] < 359.9)) {
      navigation.turnTo(2);
    } else if ((odometer.getXYT()[2] < 135 && odometer.getXYT()[2] > 90)
        || (odometer.getXYT()[2] > 45 && odometer.getXYT()[2] < 90)) {
      navigation.turnTo(90);
    } else if ((odometer.getXYT()[2] < 225 && odometer.getXYT()[2] > 180)
        || (odometer.getXYT()[2] > 135 || odometer.getXYT()[2] < 180)) {
      navigation.turnTo(180);
    } else if ((odometer.getXYT()[2] > 270 || odometer.getXYT()[2] < 315)
        && (odometer.getXYT()[2] < 270 && odometer.getXYT()[2] > 225)) {
      navigation.turnTo(270);
    }

    leftMotor.stop(true);
    rightMotor.stop();

    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * This method returns the current left color sensor reading
   * 
   * @return The current color value of the left color sensor
   */
  private float fetchLeftLightSample() {

    leftcolorSensorValue.fetchSample(leftcolorSample, 0);
    return (leftcolorSample[0] * 1000);
  }

  /**
   * This method returns the current right color sensor reading
   * 
   * @return The current color value of the right sensor
   */
  private float fetchRightLightSample() {

    rightcolorSensorValue.fetchSample(rightcolorSample, 0);
    return (rightcolorSample[0] * 1000);
  }

}