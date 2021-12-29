package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.lab5.Resources.*;


public class LightLocalizer implements Runnable {


  double[] angleUpdate = new double[4];
  SampleProvider sensorDetect = colorSensor.getMode("Red");
  float[] intensity = new float[colorSensor.sampleSize()]; // the updated fetched color sensor data
  Navigation nav = new Navigation();
  private float curIntensity;

  /**
   * Converts a distance in centimeters to the corresponding wheel rotations required
   * 
   * @param distance (cm)
   * @return distance in wheel rotations
   */
  private static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * returns the current location of the ultrasonic sensor
   * 
   * @param angle: difference between angle on one line
   * @return: coordinate location (x or y)
   */
  public static double currentSensorLocation(double angle) {
    return -Math.abs(SENSOR_LOCATION * Math.cos(Math.toRadians(angle / 2)));
  }

  /**
   * Use the light sensor to localize the origin
   */
  public void run() {

    int lineDetected = 0;// Count how many lines we've detected thus far.
    double currentX;
    double currentY;
    double angleX;
    double angleY;


    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    // Go near the origin, so that we can find the lines there
    goToOrigin();

    // Detect all 4 lines that meet at the origin
    while (lineDetected < 4) {
      // Rotate in place to find the next line.
      leftMotor.backward();
      rightMotor.forward();
      curIntensity = fetchLightSample(); // Get the intensity of the floor

      // Black line at 0.3 intensity.
      // Blue at 0.35 intensity.
      if (curIntensity < 0.3) {
        // using array to save updated theta
        angleUpdate[lineDetected] = odometer.getXYT()[2];
        lineDetected++;
        Sound.beep();
      }
    }

    leftMotor.stop(true);
    rightMotor.stop();

    // Find out our angle using the difference of the lines.
    angleY = angleUpdate[2] - angleUpdate[0]; // Lines 0 and 2 detected are the vertical lines
    angleX = angleUpdate[3] - angleUpdate[1]; // Lines 1 and 3 detected are the horizontal lines

    // Get dx and dy
    currentX = currentSensorLocation(angleY);
    currentY = currentSensorLocation(angleX);

    // Sleep for 1 second
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // Go to origin
    odometer.setXYT(currentX, currentY, odometer.getXYT()[2]); // updates odo with current location compared to origin
    nav.travelTo1(0.0, 0.0); // make it travel to origin

    // Turn to face 0 degrees
    double currAngle = odometer.getXYT()[2];
    if (currAngle <= 355 && currAngle >= 5.0) {
      nav.turnTo1(-10); // robot always under turns to right
    }

    // Sleep for 1 second
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    
    
    
    double disToTravel = Math.sqrt(Math.pow((odometer.getXYT()[0] - targetLocation[0]), 2) + Math.pow((odometer.getXYT()[1] - targetLocation[1]), 2));
   // after setup at (0,0), check if robot is in range
    if (isRobotinRange(disToTravel)) { 
      //System.out.println("robot not in cricle");
     // double Theta = Math.sqrt(Math.pow((odometer.getXYT()[0] - targetLocation[0]), 2) + Math.pow((odometer.getXYT()[1] - targetLocation[1]), 2));
    	//	  nav.turnTo(Theta);
      nav.checkinRange(true, disToTravel);
      nav.traveltoXY(targetLocation[0], targetLocation[1]);
    } else {
      System.out.println("robit is in circle");
      double[] location = newNavigatePoint();
      System.out.println("Target location"+location[0]+"   "+location[1]);
      nav.travelTo(location[0], location[1]); //travel to the new location
      
      nav.traveltoXY(targetLocation[0], targetLocation[1]);// face the new location
      
    }
  
    
    //launch the ball
    Launcher.launchPosition();
    try {
		Thread.sleep(5000);
	} catch (InterruptedException e1) {
		// TODO Auto-generated catch block
		e1.printStackTrace();
	}
    Launcher.launchBall();
    
    try {
		Thread.sleep(6000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
    Launcher.launchPosition();

  }


/**
 * the method if the robot is in the circle, it goes to one of new four navigated points 
 * @return
 */
  private double[] newNavigatePoint() {
    double [] location = new double[2];
    if(targetLocation[0] + 121.92 > 8*30.48 ) {
    if(targetLocation[0] -121.92 > 0) {
      location[0] = targetLocation[0] - 122 ;
      location[1] = targetLocation[1] ;
    }else if(targetLocation[1] +121.92 > 8*30.48) {
      location[1] = targetLocation[1] + 122 ;
      location[0] = targetLocation[0] ;
    }else if(targetLocation[1] - 121.92 > 0) {
      location[1] = targetLocation[1] - 122 ;
      location[0] = targetLocation[0] ;
    }
   }else {
     location[0] = targetLocation[0] + 121.92 ;
     location[1] = targetLocation[1] ;
   }
    return location;
  }

  
  /**
   * this method check if robot is in range
   * @param result
   * @return if robot is in range
   */
  private boolean isRobotinRange(double result) {
    if (result<125) {
      return false;
    }else {
      return true;
    }
  }
  /**
   * @return the intensity of the light sensor
   */
  private float fetchLightSample() {

    sensorDetect.fetchSample(intensity, 0);
    return intensity[0];
  }

  /**
   * This method moves the robot towards the origin
   */
  public void goToOrigin() {

	  nav.turnTo1(45);
      leftMotor.setSpeed(ROTATE_SPEED+50);
      rightMotor.setSpeed(ROTATE_SPEED+50);
      
      // Get the current intensity
      curIntensity = fetchLightSample();
      
      // Drive towards the origin until we hit a line
      while (curIntensity > 0.3) { 
          curIntensity = fetchLightSample();
          leftMotor.forward();
          rightMotor.forward();
      }
      
      leftMotor.stop(true);
      rightMotor.stop();
      Sound.beep();
      
      // Move backwards to put the rotation point on the line
      leftMotor.rotate(convertDistance(-SENSOR_LOCATION), true);
      rightMotor.rotate(convertDistance(-SENSOR_LOCATION), false);
  }
  public static double convertToDegree(double theta) {
	  return theta*180*Math.PI;
  }
}
