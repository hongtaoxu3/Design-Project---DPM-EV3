package ca.mcgill.ecse211.lab5;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import static ca.mcgill.ecse211.lab5.Resources.*;

/**
 * This class implements an Odometer thread, which updates the robot's
 * X, Y and theta coordinates as it moves.
 * 
 * @author Tony
 *
 */

public class Odometer implements Runnable {

  // Odometer update period
  private static final long ODOMETER_PERIOD = 25;

  //Position parameters
  private volatile double x;
  private volatile double y; 
  private volatile double theta;
  private static Odometer odo; 

  //Fair lock for concurrent writing
  private static Lock lock = new ReentrantLock(true);
  //Indicates if a thread is trying to reset any position parameters
  private volatile boolean isResetting = false;
  //Lets other threads know that a reset operation is over.
  private Condition doneResetting = lock.newCondition();

  // Motor-related variables
  private static int leftMotorTachoCount = 0;
  private static int rightMotorTachoCount = 0;

  private static int leftMotorLastTachoCount = 0;
  private static int rightMotorLastTachoCount = 0;

  private static double leftMotorArcLength = 0;
  private static double rightMotorArcLength = 0;

  // Update motor related variable, deltaC and deltaTheta
  private static double deltaT;
  private static double deltaD;

  // new position of robot
  private static double dTheta;
  private static double dX;
  private static double dY;

  /**
   * Default constructor of the odometer. The odometer is initialized
   * at (0, 0, 0).
   * 
   */

  private Odometer() {
    setXYT(0, 0, 0);
  }

  /**
   * Method that returns an Odometer object, use 
   * this method to obtain an instance of Odometer.
   * 
   * @return an Odometer object
   */
  public synchronized static Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }
    return odo;
  }

  /**
   * This method constantly updates the coordinates of the current
   * position of the robot.
   * 
   */

  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorLastTachoCount = leftMotorTachoCount;
      rightMotorLastTachoCount = rightMotorTachoCount;

      // get the current tachometer count
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      // new arc length distance to be updated by each wheel
      leftMotorArcLength = Math.PI * WHEEL_RAD * (leftMotorTachoCount - leftMotorLastTachoCount) / 180;
      rightMotorArcLength = Math.PI * WHEEL_RAD * (rightMotorTachoCount - rightMotorLastTachoCount) / 180;

      // average of left and right distance (arc length travelled by center of distance)
      deltaD = 0.5 * (leftMotorArcLength + rightMotorArcLength);

      // new angle to be updated
      deltaT = ((leftMotorArcLength - rightMotorArcLength) / TRACK) * 180 / Math.PI;

      dTheta = odo.getXYT()[2];

      // calculate the center arc length
      dTheta += deltaT;

      // new distance to be updated, sine function for x direction and cosine for y direction
      dX = deltaD * Math.sin(Math.toRadians(dTheta));
      dY = deltaD * Math.cos(Math.toRadians(dTheta));
      // Update odometer values with new calculated values
      odo.update(dX, dY, deltaT);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  /**
   * This method returns the Odometer data.
   * <p>
   * Writes the current position and orientation of the robot onto the odoData array. {@code odoData[0] =
   * x, odoData[1] = y; odoData[2] = theta;}
   * 
   * @return the odometer readings
   */
  public double[] getXYT() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;
  }

  /**
   * This method adds dx, dy and dtheta to the current values of x, y and theta, respectively
   * 
   * @param dx x update
   * @param dy y update
   * @param dtheta theta update
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isResetting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates within 360 degrees
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock();
    }
  }

  /**
   * This method overrides the values of x, y and theta
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setXYT(double x, double y, double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }
}


