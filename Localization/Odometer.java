package ca.mcgill.ecse211.lab4;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import static ca.mcgill.ecse211.lab4.Resources.*;

/**
 * The odometer class keeps track of the robot's (x, y, theta) position.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 * @author Younes Boubekeur
 */

public class Odometer implements Runnable {
  
  /**
   * The x-axis position in cm.
   */
  private volatile double x;
  
  /**
   * The y-axis position in cm.
   */
  private volatile double y; // y-axis position
  
  /**
   * The orientation in degrees.
   */
  private volatile double theta; // Head angle
  
  /**
   * The (x, y, theta) position as an array
   */
  private double[] position;

  // Thread control tools
  /**
   * Fair lock for concurrent writing
   */
  private static Lock lock = new ReentrantLock(true);
  
  /**
   * Indicates if a thread is trying to reset any position parameters
   */
  private volatile boolean isResetting = false;

  /**
   * Lets other threads know that a reset operation is over.
   */
  private Condition doneResetting = lock.newCondition();

  private static Odometer odo; // Returned as singleton

  // Current tacho variables
  private static int nowLeftMotorTachoCount;
  private static int nowRightMotorTachoCount;
  
  // Previous tacho variables
  private static int lastLeftMotorTachoCount;
  private static int lastRightMotorTachoCount;

  /**
   * The odometer update period in ms.
   */
  private static final long ODOMETER_PERIOD = 25;
  
  private double leftDisplacement;
  private double rightDisplacement;
  private double deltaD;
  private double deltaT;
  private double dx;
  private double dy;

  
  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   */
  private Odometer() {
    
    setXYT(0, 0, 0);
    
    // Reset and initialize tacho counts
    
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    
    lastLeftMotorTachoCount = 0;
    lastRightMotorTachoCount = 0;
  }

  /**
   * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
   * 
   * @return the Odometer Object
   */
  public synchronized static Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }
    
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run.
   */
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      // Get current tacho counts, set it to another variable
      
      nowLeftMotorTachoCount = leftMotor.getTachoCount();
      nowRightMotorTachoCount = rightMotor.getTachoCount();

      // Calculate left and right wheel displacements using formula from the slides
      
      leftDisplacement = Math.PI * WHEEL_RAD * (nowLeftMotorTachoCount - lastLeftMotorTachoCount) / 180;
      rightDisplacement = Math.PI * WHEEL_RAD * (nowRightMotorTachoCount - lastRightMotorTachoCount) / 180;
      
      // Update last tacho counts
      
      lastLeftMotorTachoCount = nowLeftMotorTachoCount;
      lastRightMotorTachoCount = nowRightMotorTachoCount;
      
      // Calculate the vehicle displacement
      
      deltaD = 0.5 * (leftDisplacement + rightDisplacement);
      
      // Calculate the change in heading (converted rad to degree)
      
      deltaT = ((leftDisplacement - rightDisplacement) / TRACK) * 180 / Math.PI;
      
      // Get current position (x, y, theta)
      
      this.position = this.getXYT();
      
      // Compute dx, dy, dtheta
      
      dx = deltaD * Math.sin(this.position[2] * Math.PI / 180);
      dy = deltaD * Math.cos(this.position[2] * Math.PI / 180);
      
      // Update odometer values with new calculated values
      
      odo.update(dx, dy, deltaT);

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
  
  // IT IS NOT NECESSARY TO MODIFY ANYTHING BELOW THIS LINE
  
  /**
   * Returns the Odometer data.
   * <p>
   * Writes the current position and orientation of the robot onto the odoData array. {@code odoData[0] =
   * x, odoData[1] = y; odoData[2] = theta;}
   * 
   * @param position the array to store the odometer data
   * @return the odometer data.
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
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for
   * odometry.
   * 
   * @param dx
   * @param dy
   * @param dtheta
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
   * Overrides the values of x, y and theta. Use for odometry correction.
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
