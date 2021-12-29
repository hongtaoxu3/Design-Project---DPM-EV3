package ca.mcgill.ecse211.lab5;


import static ca.mcgill.ecse211.lab5.Resources.*;
import lejos.hardware.Sound;

public class Navigation {
  private static boolean traveling;
  private static double currentPosition[] = new double[3];
  //private static double currentTheta; 

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   *
   * @param distance: distance needed to travel based on wheel radius
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   *
   * @param angle: angle to rotate
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }

  /**
   * this method is we used for the robot travel to target position, we used euclidean distance 
   * error to compute the distance we need to travel
   *
   * @param x, y coordinates of next point
   */
  public void travelTo(double x, double y) {
      
      //Get initial coordinates
      double currentX = odometer.getXYT()[0];
      double currentY = odometer.getXYT()[1];
      double currentTheta = 0;
      
      if (currentX == x) {
          if (currentY > y) {
              currentTheta = Math.PI;
          } else if (currentY < y) {
              currentTheta = 0;
          }
     
      } else if (currentY == y) {
          if (currentX > x) {
              currentTheta = -Math.PI / 2;
          } else if (currentX < x) {
              currentTheta = Math.PI / 2;
          }
      }
      else {
          currentTheta = Math.atan((currentX - x) / (currentY - y));
          if (currentY > y) {
              currentTheta += Math.PI;
          }
      }
      
      turnTo(Math.toDegrees(currentTheta)); // Turn the robot towards the destination
      
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
  
		//Compute the distance to destination
		double distToTravel = euclideanDistance(currentX, currentY, x, y);
		
		//Drive towards the destination
		rightMotor.rotate(convertDistance(distToTravel), true);
		leftMotor.rotate(convertDistance(distToTravel), false);
		Sound.beep();
		
  }
  
	/**
	 * 
	 * Turns the robot in the correct direction to get to the next way-point. Ensures that the rotation is always a
   * minimum angle.
	 *
	 * @param theta: angle wanted to rotate to
	 */
	public void turnTo(double theta) {
		double dTheta = getAngle(theta);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		rightMotor.rotate(-convertAngle(dTheta), true); 
		leftMotor.rotate(convertAngle(dTheta), false);
	}
  
  
	/**
	 * This method will return the euclidean distance of the robot with respect to the current destination
	 *
	 * @param currentX, Y coordinates and target X,Y coordinate.
	 * @return euclidean error 
	 */
	private double euclideanDistance(double currentX, double currentY, double targX, double targY) {
		return Math.sqrt(Math.pow((currentX - targX), 2) + Math.pow((currentY - targY), 2));
	}
	
  
  /**
   * method is the travelTo method we used for normal localization
   * 
   * @param nextX the next X position
   * @param nextY the next Y position
   */
  public void travelTo1(double nextX, double nextY) {
    // get the current position of the robot
    currentPosition = odometer.getXYT();

    // calculate the trajectory required to get to next way-point
    double deltaX = nextX - currentPosition[0];
    double deltaY = nextY - currentPosition[1];
    double nextTheta = (Math.atan2(deltaX, deltaY)) * 180 / Math.PI;
    double trajectory = Math.hypot(deltaX, deltaY);

    // travel to next way point
    traveling = true;
    turnTo1(nextTheta); // turn in correct direction

    // move forward
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.rotate(convertDistance(trajectory), true);
    leftMotor.rotate(convertDistance(trajectory), false);

    // now that we are done, set traveling to false
    traveling = false;
        

  }
  
  
  
  /**this method check if the robot is place out of the 120cm radius, 
   * if is out of radius do euclidean distance travel and minus 120,
   * otherwise do travel out 
   * @param shootingrange(boolean) and travel distance
   */
  public void checkinRange(boolean outShootingRange, double travelDis) {

    if (outShootingRange) { 
        System.out.println("robot outside the range, travel towards targe");    
        if (travelDis > 0) {
        	//System.out.println("travelDis-RADIUS_TARGET_LOCATION "+(travelDis-RADIUS_TARGET_LOCATION) );
        	rightMotor.setSpeed(FORWARD_SPEED);
        	leftMotor.setSpeed(FORWARD_SPEED);
        	rightMotor.rotate(convertDistance(travelDis-RADIUS_TARGET_LOCATION ),true);
        leftMotor.rotate(convertDistance(travelDis-RADIUS_TARGET_LOCATION),false);
        }
    } else {
      System.out.println("towards out travel");    
      rightMotor.rotate(convertDistance(travelDis), true);
      leftMotor.rotate(convertDistance(travelDis), false);
    }
    Sound.beep();
    }

  
  
  /**
   * 
   * method that is normal turnTo method we used in localization
   * @param theta (deg)
   */

  void turnTo1(double theta) {
    // determine the correction in angle required
    currentPosition = odometer.getXYT();
    double error = theta - currentPosition[2];
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
   * @param theta (deg)
   */
  void turnLeft(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(-convertAngle(Math.abs(theta-10)), true);
    rightMotor.rotate(convertAngle(Math.abs(theta-10)), false);
  }

  /**
   * @param theta (deg)
   */
  void turnRight(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(Math.abs(theta-10)), true);
    rightMotor.rotate(-convertAngle(Math.abs(theta-10)), false);
  }

  /**
   * @return true if we are navigating
   */
  static boolean isNavigating() {
    return traveling;
  }
  
  
  /**
   * this method calculate the turning angle that robot should turn after it's travel to 
   * face to the target position
   * @param X,Y coordinate
   * @return true if we are navigating
   */
  void traveltoXY(double x, double y) {
      //Get initial coordinates
      double currentX = odometer.getXYT()[0];
      double currentY = odometer.getXYT()[1];
      double currentTheta = 0;
      
      //Calculate angle needed to turn
      if (currentX == x) {
          if (currentY > y) {
              currentTheta = Math.PI;
          } else if (currentY < y) {
              currentTheta = 0;
          }
         
      } else if (currentY == y) {
          if (currentX > x) {
              currentTheta = -Math.PI / 2;
          } else if (currentX < x) {
              currentTheta = Math.PI / 2;
          }
      }
      else {
          currentTheta = Math.atan((currentX - x) / (currentY - y));
          if (currentY > y) {
              currentTheta += Math.PI;
          }
      }
      
      double currtheta = Math.toDegrees(currentTheta);
      double deltaTheta = getAngle(currtheta);
      
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      
      rightMotor.rotate(-convertAngle(deltaTheta-28), true);
      leftMotor.rotate(convertAngle(deltaTheta-28), false);
      
  }

  /**
   * this method calculate the angle that robot needs to turn to the face
   * the target position
   * @param angle theta which is the current theta
   * @return the turning angle 
   */
  private double getAngle(double theta) {
      double currentTheta = odometer.getXYT()[2];
      //Computes the angle between 0 and 360
      double dTheta = (theta - currentTheta + 360) % 360;
      
      //Converts to minimum angle
      if (Math.abs(dTheta - 360) < dTheta) {
          dTheta -= 360;
      }
      
      return dTheta;
  }

  

}

