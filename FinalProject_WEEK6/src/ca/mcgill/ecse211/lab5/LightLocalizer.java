package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import static ca.mcgill.ecse211.lab5.Resources.*;

/**
 * This class performs localization using the right color sensor to locate and
 * position the robot to the north with angle 0.
 * 
 * @author Tony Ou
 * @author Xinran Li
 *
 */

public class LightLocalizer {

  double[] currentPosition = new double[3];
  double[] angleMeasures = new double[4];
  Navigation navigation = new Navigation();

  /**
   * This method returns the current location of the color sensor
   * 
   * @param angle The angle difference on one line
   * @return The location of the US sensor
   */

  public static double currentSensorLocation(double angle) {
    return -Math.abs(SENSOR_LOCATION * Math.cos(Math.toRadians(angle / 2)));
  }

  /**
   * This method starts the light localization. It first positions the robot to an
   * approximation of the origin (1,1). It then scans all 4 grid lines to
   * calculate the real coordinates of the origin. Finally, the robot corrects its
   * position and orientation. After localization is performed, the robot should
   * be pointing north on the point (1,1).
   * 
   */

  public void localize() {

    int lineDetected = 0;// Count how many lines we've detected thus far.
    double currentX;
    double currentY;
    double angleX;
    double angleY;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    goToOrigin();

    float lastColorValue = 0;

    // Detect all 4 lines that meet at the origin
    while (lineDetected < 4) {
      // Rotate in place to find the next line.
      leftMotor.backward();
      rightMotor.forward();
      float colorValueLines = fetchrightLightSample();
      float difference = colorValueLines - lastColorValue;
      lastColorValue = colorValueLines;
      if (difference < -20) {
        // using array to save updated theta
        angleMeasures[lineDetected] = odometer.getXYT()[2];
        // System.out.println("detect theta: " + angleMeasures[lineDetected]);
        lineDetected++;

        // Sleep for 400 miliseconds
        try {
          Thread.sleep(500);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }

      }
    }

    leftMotor.stop(true);
    rightMotor.stop();

    // Find out our angle using the difference of the lines.
    angleY = angleMeasures[2] - angleMeasures[0]; // Lines 0 and 2 detected are the vertical lines
    angleX = angleMeasures[3] - angleMeasures[1]; // Lines 1 and 3 detected are the horizontal line
    // Get dx and dy
    currentX = currentSensorLocation(angleY);
    currentY = currentSensorLocation(angleX);

    // Sleep for 10 miliseconds
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // set the corrected odometer
    odometer.setXYT(currentX, currentY, odometer.getXYT()[2]); // updates odo with current location compared to
    // origin
    navigation.travelTo(0.0, 0.0); // make it travel to origin

    // Turn to face 0 degrees
    //		double currAngle = odometer.getXYT()[2];
    //		System.out.println("currentAngle" +currAngle);
    //		if (currAngle <= 355 && currAngle >= 5.0) {
    //			
    //			navigation.turnTo(9.7);
    //		}

    navigation.turnTo(20.0);

    // decide the new odometer

    if (TEAM_NUMBER == greenTeam) {
      System.out.println("go into greenteam");
      switch (greenCorner) {
        case 0:
          odometer.setXYT(TILE_SIZE, TILE_SIZE, 0.0);
          currentPosition = odometer.getXYT();
          System.out.println(
              "currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 1:
          odometer.setXYT(TILE_SIZE * 14, TILE_SIZE, 270.0);
          currentPosition = odometer.getXYT();
          System.out.println(
              "currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 2:
          odometer.setXYT(TILE_SIZE * 14, TILE_SIZE * 8, 180.0);
          currentPosition = odometer.getXYT();
          System.out.println(
              "currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 3:
          odometer.setXYT(TILE_SIZE, TILE_SIZE * 8, 90.0);
          currentPosition = odometer.getXYT();
          System.out.println(
              "currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
      }
    }

    else if (TEAM_NUMBER == redTeam) {
      System.out.println("go into redteam");
      switch (redCorner) {
        case 0:
          odometer.setXYT(TILE_SIZE, TILE_SIZE, 0.0);
          currentPosition = odometer.getXYT();
          System.out.println(
              "currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 1:
          odometer.setXYT(TILE_SIZE * 14, TILE_SIZE, 270.0);
          currentPosition = odometer.getXYT();
          System.out.println(
              "currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 2:
          odometer.setXYT(TILE_SIZE * 14, TILE_SIZE * 8, 180.0);
          currentPosition = odometer.getXYT();
          System.out.println(
              "currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 3:
          odometer.setXYT(TILE_SIZE, TILE_SIZE * 8, 90.0);
          currentPosition = odometer.getXYT();
          System.out.println(
              "currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
      }
    }

    Sound.beep();
    Sound.beep();
    Sound.beep();

    try {
      Thread.sleep(300);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    traveltoTunnel();

  }

  /*
   * This method drives the robot to face the tunnel
   *
   */

  public void traveltoTunnel() {
    // navigation.travelTo(3,3);
    System.out.println("in travelToTunnel ");

    // if we are green team
    if (greenTeam == TEAM_NUMBER) {

      if (greenCorner == 0) {
        System.out.println("green0");
        if (tng.ur.y - tng.ll.y == 1) {
          // horizontal横着放->
          navigation.travelTo(tng.ll.x - 1, tng.ll.y + 0.65);
          System.out.println(tng.ll.x - 1);
          System.out.println(tng.ll.y + 0.65);
          navigation.turnTo(90);
          // TODO
        } else if (tng.ur.x - tng.ll.x == 1) {
          // perpendicular竖着放 个
          navigation.travelTo(tng.ll.x + 0.5, tng.ll.y - 1);
          System.out.println(tng.ll.x + 0.5);
          System.out.println(tng.ll.y - 1);
          navigation.turnTo(0);
          // TODO
        }
      } else if (greenCorner == 1) {
        System.out.println("green1");
        if (tng.ur.y - tng.ll.y == 1) {
          // horizontal横着放<-
          navigation.travelTo(tng.ur.x + 1, tng.ur.y - 0.5);
          System.out.println(tng.ur.x + 1);
          System.out.println(tng.ur.y - 0.5);
          navigation.turnTo(270);
          // TODO
        } else if (tng.ur.x - tng.ll.x == 1) {
          // perpendicular竖着放个
          navigation.travelTo(tng.ll.x + 0.5, tng.ll.y - 1);
          System.out.println(tng.ll.x + 0.8);
          System.out.println(tng.ll.y - 1);
          navigation.turnTo(0);
          // TODO
        }
      } else if (greenCorner == 2) {
        System.out.println("green2");
        if (tng.ur.y - tng.ll.y == 1) {
          // horizontal横着放
          navigation.travelTo(tng.ur.x + 1, tng.ur.y - 0.5);
          System.out.println(tng.ur.x + 1);
          System.out.println(tng.ur.y - 0.5);
          navigation.turnTo(270);
          // TODO
        } else if (tng.ur.x - tng.ll.x == 1) {
          // perpendicular竖着放
          navigation.travelTo(tng.ll.x - 0.5, tng.ll.y + 1);
          System.out.println(tng.ll.x - 0.5);
          System.out.println(tng.ll.y + 1);
          navigation.turnTo(180);
          // TODO
        } else if (greenCorner == 3) {
          System.out.println("green3");
          if (tng.ur.y - tng.ll.y == 1) {
            // horizontal横着放
            navigation.travelTo(tng.ll.x - 1, tng.ll.y + 0.5);
            System.out.println(tng.ur.x - 1);
            System.out.println(tng.ur.y + 0.5);
            navigation.turnTo(90);
            // TODO
          } else if (tng.ur.x - tng.ll.x == 1) {
            // perpendicular竖着放
            navigation.travelTo(tng.ur.x - 0.5, tng.ur.y + 1);
            System.out.println(tng.ur.x - 0.5);
            System.out.println(tng.ur.y + 1);
            navigation.turnTo(180);
            // TODO
          }
        }
      }

    }

    else if (redTeam == TEAM_NUMBER) {

      if (redCorner == 0) {
        System.out.println("red0");
        if (tnr.ur.y - tnr.ll.y == 1) {
          // horizontal横着放
          navigation.travelTo(tnr.ll.x - 1, tnr.ll.y + 0.5);
          System.out.println(tnr.ll.x - 1);
          System.out.println(tnr.ll.y + 0.5);
          navigation.turnTo(90);
          // TODO
        } else if (tnr.ur.x - tnr.ll.x == 1) {
          // perpendicular竖着放
          // ****************************************************
          navigation.travelTo(tnr.ll.x + 0.5, tnr.ll.y - 1.7);
          System.out.println(tnr.ll.x + 0.5);
          System.out.println(tnr.ll.y - 1.7);
          navigation.turnTo(0);
          // TODO
        }
      } else if (redCorner == 1) {
        System.out.println("red1");
        if (tnr.ur.y - tnr.ll.y == 1) {
          // horizontal横着放
          navigation.travelTo(tnr.ur.x + 1, tnr.ur.y - 0.5);
          System.out.println(tnr.ur.x + 1);
          System.out.println(tnr.ur.y - 0.5);
          navigation.turnTo(270);
          // TODO
        } else if (tnr.ur.x - tnr.ll.x == 1) {
          // perpendicular竖着放
          navigation.travelTo(tnr.ll.x + 0.5, tnr.ll.y - 1);
          System.out.println(tnr.ll.x + 0.5);
          System.out.println(tnr.ll.y - 1);
          navigation.turnTo(0);
          // TODO
        }
      } else if (redCorner == 2) {
        System.out.println("red2");
        if (tnr.ur.y - tnr.ll.y == 1) {

          // horizontal横着放
          navigation.travelTo(tnr.ur.x + 1, tnr.ur.y - 0.5);
          System.out.println(tnr.ur.x + 1);
          System.out.println(tnr.ur.y - 0.5);
          navigation.turnTo(270);
          // TODO
        } else if (tnr.ur.x - tnr.ll.x == 1) {
          // perpendicular竖着放
          navigation.travelTo(tnr.ll.x - 0.5, tnr.ll.y + 1);
          System.out.println(tnr.ll.x - 0.5);
          System.out.println(tnr.ll.y + 1);
          navigation.turnTo(180);
          // TODO
        }
      } else if (redCorner == 3) {
        System.out.println("red3");
        if (tnr.ur.y - tnr.ll.y == 1) {
          // horizontal横着放
          navigation.travelTo(tnr.ll.x - 1, tnr.ll.y + 0.5);
          System.out.println(tnr.ll.x - 1);
          System.out.println(tnr.ll.y + 0.5);
          navigation.turnTo(90);
          // TODO
        } else if (tnr.ur.x - tnr.ll.x == 1) {
          // perpendicular竖着放
          navigation.travelTo(tnr.ur.x - 0.5, tnr.ur.y + 1);
          System.out.println(tnr.ur.x - 0.5);
          System.out.println(tnr.ur.y + 1);
          navigation.turnTo(180);
          // TODO
        }
      }

    }

  }

  /*
   * This method drives the robot through the tunnel to the middle of tile after
   * pass through the tunnel
   */
  public void travelthrough() {

    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.rotate(navigation.convertDistance(3.5 * TILE_SIZE), true);
    leftMotor.rotate(navigation.convertDistance(3.5 * TILE_SIZE), false);

    rightMotor.stop(true);
    leftMotor.stop();

  }

  /*
   * this method drives robot back to the front of tunnel after launching
   */
  public void goBackfront() {
    // green team
    if (TEAM_NUMBER == greenTeam) {
      switch (greenCorner) {
        case 0:
          if (tng.ur.y - tng.ll.y == 1) {
            // horizontal横着放->
            navigation.travelTo(tng.ur.x + 1, tng.ur.y - 0.35);
            navigation.turnTo(270);

            // TODO
          } else if (tng.ur.x - tng.ll.x == 1) {
            // perpendicular竖着放 个
            navigation.travelTo(tng.ll.x - 0.5, tng.ll.y + 1);
            navigation.turnTo(180);

            // TODO
          }
          break;
        case 1:
          if (tng.ur.y - tng.ll.y == 1) {
            // horizontal横着放->
            navigation.travelTo(tng.ll.x - 1, tng.ll.y + 0.5);
            navigation.turnTo(90);

            // TODO
          } else if (tng.ur.x - tng.ll.x == 1) {
            // perpendicular竖着放 个
            navigation.travelTo(tng.ur.x - 0.77, tng.ur.y + 1);
            navigation.turnTo(180);

            // TODO
          }
          break;
        case 2:
          if (tng.ur.y - tng.ll.y == 1) {
            // horizontal横着放->
            navigation.travelTo(tng.ll.x - 1, tng.ll.y + 0.5);
            navigation.turnTo(90);

            // TODO
          } else if (tng.ur.x - tng.ll.x == 1) {
            // perpendicular竖着放 个
            navigation.travelTo(tng.ll.x + 0.5, tng.ll.y - 1);
            navigation.turnTo(0);

            // TODO
          }
          break;
        case 3:
          if (tng.ur.y - tng.ll.y == 1) {
            // horizontal横着放->
            navigation.travelTo(tng.ur.x + 1, tng.ur.y - 0.5);
            navigation.turnTo(270);

            // TODO
          } else if (tng.ur.x - tng.ll.x == 1) {
            // perpendicular竖着放 个
            navigation.travelTo(tng.ll.x + 0.5, tng.ll.y - 1);
            navigation.turnTo(0);

            // TODO
          }
          break;
      }
    }

    // red team
    else if (TEAM_NUMBER == redTeam) {
      switch (redTeam) {
        case 0:
          if (tnr.ur.y - tnr.ll.y == 1) {
            // horizontal横着放->
            navigation.travelTo(tnr.ur.x + 1, tnr.ur.y - 0.5);
            navigation.turnTo(270);

            // TODO
          } else if (tnr.ur.x - tnr.ll.x == 1) {
            // perpendicular竖着放 个
            navigation.travelTo(tnr.ll.x - 0.5, tnr.ll.y + 1);
            navigation.turnTo(180);

            // TODO
          }
          break;
        case 1:
          if (tnr.ur.y - tnr.ll.y == 1) {
            // horizontal横着放->
            navigation.travelTo(tnr.ll.x - 1, tnr.ll.y + 0.5);
            navigation.turnTo(90);

            // TODO
          } else if (tnr.ur.x - tnr.ll.x == 1) {
            // perpendicular竖着放 个
            navigation.travelTo(tnr.ur.x - 0.5, tnr.ur.y + 1);
            navigation.turnTo(180);
            // TODO
          }
          break;
        case 2:
          if (tnr.ur.y - tnr.ll.y == 1) {
            // horizontal横着放->
            navigation.travelTo(tnr.ll.x - 1, tnr.ll.y + 0.5);
            navigation.turnTo(90);

            // TODO
          } else if (tnr.ur.x - tnr.ll.x == 1) {
            // perpendicular竖着放 个
            navigation.travelTo(tnr.ll.x + 0.5, tnr.ll.y - 1);
            navigation.turnTo(0);

            // TODO
          }
          break;
        case 3:
          if (tnr.ur.y - tnr.ll.y == 1) {
            // horizontal横着放->
            navigation.travelTo(tnr.ur.x + 1, tnr.ur.y - 0.5);
            navigation.turnTo(270);
            // TODO
          } else if (tnr.ur.x - tnr.ll.x == 1) {
            // perpendicular竖着放 个
            navigation.travelTo(tnr.ll.x + 0.5, tnr.ll.y - 1);
            navigation.turnTo(0);
            // TODO
          }
          break;
      }
    }

  }

  /*
   * this method drives robot back to the corner after it arrives on its own
   * colored island
   */
  public void backToCorner() {
    if (TEAM_NUMBER == greenTeam) {
      System.out.println("go into greenteam");
      switch (greenCorner) {
        case 0:
          navigation.travelTo(TILE_SIZE, TILE_SIZE);
          //			currentPosition = odometer.getXYT();
          //			System.out.println(
          //					"currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 1:
          navigation.travelTo(TILE_SIZE * 14, TILE_SIZE);
          //			currentPosition = odometer.getXYT();
          //			System.out.println(
          //					"currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 2:
          navigation.travelTo(TILE_SIZE * 14, TILE_SIZE * 8);
          //			currentPosition = odometer.getXYT();
          //			System.out.println(
          //					"currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 3:
          navigation.travelTo(TILE_SIZE, TILE_SIZE * 8);
          //			currentPosition = odometer.getXYT();
          //			System.out.println(
          //					"currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
      }
    }

    else if (TEAM_NUMBER == redTeam) {
      System.out.println("go into redteam");
      switch (redCorner) {
        case 0:
          navigation.travelTo(TILE_SIZE, TILE_SIZE);
          //			currentPosition = odometer.getXYT();
          //			System.out.println(
          //					"currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 1:
          navigation.travelTo(TILE_SIZE * 14, TILE_SIZE);
          //			currentPosition = odometer.getXYT();
          //			System.out.println(
          //					"currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 2:
          navigation.travelTo(TILE_SIZE * 14, TILE_SIZE * 8);
          //			currentPosition = odometer.getXYT();
          //			System.out.println(
          //					"currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
        case 3:
          navigation.travelTo(TILE_SIZE, TILE_SIZE * 8);
          //			currentPosition = odometer.getXYT();
          //			System.out.println(
          //					"currentPosition: " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2]);
          break;
      }
    }
  }

  /*
   * This method returns the current right color sensor reading
   * 
   * @return The current color value of the right color sensor
   */
  private float fetchrightLightSample() {

    rightcolorSensorValue.fetchSample(rightcolorSample, 0);
    return (rightcolorSample[0] * 1000);
  }

  /**
   * This method positions the robot approximately on the origin point (1,1). The
   * robot turns 45 degrees and moves forward until the color sensor detects a
   * line. It then travels backwards to compensate for the color sensor's
   * position.
   * 
   */

  public void goToOrigin() {

    navigation.turnTo(45);
    // navigation2.turnTo(45);

    // Drive towards the origin until we hit a line
    leftMotor.forward();
    rightMotor.forward();

    float lastColorValue = 0;

    while (true) {
      // Get the current intensity
      float colorValueOrigin = fetchrightLightSample();
      float difference = colorValueOrigin - lastColorValue;
      lastColorValue = colorValueOrigin;
      if (difference < -20) {
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        // Sound.beep();
        leftMotor.stop(true);
        rightMotor.stop();
        break;
      }
    }
    // Move backwards to put the rotation point on the line
    leftMotor.rotate(navigation.convertDistance(-SENSOR_LOCATION), true);
    rightMotor.rotate(navigation.convertDistance(-SENSOR_LOCATION), false);
  }
}
