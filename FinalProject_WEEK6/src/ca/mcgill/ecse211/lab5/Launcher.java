package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Resources.*;

/**
 * This class provides methods relevant to launching the ball
 * 
 * @author Tony
 * @author Xinran Li
 *
 */

public class Launcher {

  private static double currentPosition[] = new double[3];

  private static double targetLocation[] = new double[2]; // bin coordinate

  // NaviAvoid naviAvoid = new NaviAvoid();
  Navigation navigation = new Navigation();

  /**
   * This method makes the robot travel to the launch point, turn towards the bin and launch
   * the ball.
   */

  public void travelAndLaunch() {

    // method 1
    if (greenTeam == TEAM_NUMBER) {
      targetLocation[0] = greenBin.x;
      targetLocation[1] = greenBin.y;
    } else if (redTeam == TEAM_NUMBER) {
      targetLocation[0] = redBin.x;
      targetLocation[1] = redBin.y;
    }
    currentPosition = odometer.getXYT();
    System.out.println(
        "current position:" + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);

    boolean isAvoiding = false;

    // calculate euclidean error distance
    double disToTravel = Math.sqrt(Math.pow((currentPosition[0] - targetLocation[0]*TILE_SIZE), 2)
        + Math.pow((currentPosition[1] - targetLocation[1]*TILE_SIZE), 2));
    System.out.println("distotravel: " + disToTravel);

    // turn to face the bin
    double deltaX = targetLocation[0] * TILE_SIZE - currentPosition[0];
    double deltaY = targetLocation[1] * TILE_SIZE - currentPosition[1];
    double nextTheta = (Math.atan2(deltaX, deltaY)) * 180 / Math.PI;
    navigation.turnTo(nextTheta);

    //prey to god the bin is seven tiles away
    if (disToTravel < RADIUS_TARGET_LOCATION) {
      launchBall();
    } else {
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.rotate(navigation.convertDistance(disToTravel - RADIUS_TARGET_LOCATION), true);
      leftMotor.rotate(navigation.convertDistance(disToTravel - RADIUS_TARGET_LOCATION), true);
      System.out.println("real travel distance: " +(disToTravel - RADIUS_TARGET_LOCATION));}
    // do avoiding during travel
    while (rightMotor.isMoving() && leftMotor.isMoving()) {
      currentPosition = odometer.getXYT();

      if (ultrasonicPoller.getDistance() < 10) {
        System.out.println("111111111111 step!");
        isAvoiding = true;
        leftMotor.stop(true);
        rightMotor.stop();
        System.out.println("22222222222 step!");
        navigation.turnRight(90);
        System.out.println("33333333333333 step!");
      }

      if (isAvoiding) {
        // stop the motors
        System.out.println("444444444444 step!");
        leftMotor.stop(true);
        rightMotor.stop();
        // travel the length of the obstacle
        clearObstacle(OBSTACLE_LENGTH);
        // turn 90 degrees once we have cleared the object length
        navigation.turnLeft(90);
        // travel the width of the obstacle
        clearObstacle(OBSTACLE_WIDTH);
        while (rightMotor.isMoving() && leftMotor.isMoving()) {
          if (ultrasonicPoller.getDistance() < 10) {
            leftMotor.stop(true);
            rightMotor.stop();
            navigation.turnRight(90);
            clearObstacle(OBSTACLE_LENGTH);
            // turn 90 degrees once we have traveled the width
            navigation.turnLeft(90);
            // travel the length of the obstacle
            clearObstacle(OBSTACLE_WIDTH);
            navigation.turnLeft(90);
            clearObstacle(OBSTACLE_LENGTH);
            isAvoiding = false;
            travelAndLaunch();
          } else {
            // turn 90 degrees once we have traveled the width
            navigation.turnLeft(90);
            // travel the length of the obstacle
            clearObstacle(OBSTACLE_LENGTH);
            // we have now successfully cleared the object, resume normal motion
            isAvoiding = false;
            travelAndLaunch();
          }
        }

      }

    }

    launchBall();

  }

  // calculate the coordinate to shoot
  //	private double[] newNavigatePoint() {
  //		double[] location = new double[2];
  //		//decide x coordinate of the launching point 
  //		if (targetLocation[0] + RADIUS_TARGET_LOCATION < island.ur.x) {
  //			if (targetLocation[0] + RADIUS_TARGET_LOCATION > island.ll.x) {
  //				location[0] = targetLocation[0] + RADIUS_TARGET_LOCATION;
  //			}
  //		} else {
  //			location[0] = targetLocation[0] - RADIUS_TARGET_LOCATION;
  //		}
  //		//decide y coordinate of the launching point 
  //		if (targetLocation[1] + RADIUS_TARGET_LOCATION < island.ur.y) {
  //			if (targetLocation[1] + RADIUS_TARGET_LOCATION > island.ll.y) {
  //				location[1] = targetLocation[1] + RADIUS_TARGET_LOCATION;
  //			}
  //		} else {
  //			location[1] = targetLocation[1] - RADIUS_TARGET_LOCATION;
  //		}
  //
  //		return location;
  //	}

  /**
   * Travels in a straight line in order to avoid an obstacle, to be used after
   * turning 90 degrees
   * 
   * @param distance (cm) to be traveled
   */
  public void clearObstacle(double distance) {
    leftMotor.rotate(navigation.convertDistance(distance), true);
    rightMotor.rotate(navigation.convertDistance(distance), false);
    leftMotor.stop();
    rightMotor.stop();
  }

  /**
   * This method uses the launcher motors to launch the ball
   */

  public static void launchBall() {
    rightLauncher.setSpeed(LAUNCH_SPEED);
    leftLauncher.setSpeed(LAUNCH_SPEED);

    leftLauncher.rotate(-360, true);
    rightLauncher.rotate(-360, false);

    rightLauncher.stop();
    leftLauncher.stop();
  }
}
