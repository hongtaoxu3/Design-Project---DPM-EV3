package ca.mcgill.ecse211.lab5;


import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import static ca.mcgill.ecse211.lab5.Resources.*;


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
   * This is the default constructor of this class. It initiates all motors and variables once.It cannot be accessed
   * externally.
   */
  private Odometer() {
    setXYT(0, 0, 0);
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
   * This thread updates odometer along when it moves in a square shape. 
   * It uses the difference between the current tacho count and the last one to compute the newly travelled distance. 
   * We take the average of the left and right motor arc length to get the distance travelled by the center and use trignometry 
   * to calculate the newly travelled distance in x and y direction seperately.
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

      // TODO Calculate new robot position based on tachometer counts

      // new arc length distance to be updated by each wheel
      leftMotorArcLength = Math.PI * WHEEL_RAD * (leftMotorTachoCount - leftMotorLastTachoCount) / 180;
      rightMotorArcLength = Math.PI * WHEEL_RAD * (rightMotorTachoCount - rightMotorLastTachoCount) / 180;

      // average of left and right distance (arc length travelled by center of distance)
      deltaD = 0.5 * (leftMotorArcLength + rightMotorArcLength);

      // new angle to be updated
      deltaT = ((leftMotorArcLength - rightMotorArcLength) / TRACK) * 180 / Math.PI;
      
      // theta =odo.getXYT()[2];
      dTheta = odo.getTheta();
      
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
   * Returns the Odometer data X.
   * <p>
   * 
   * @return the odometer data X.
   */
  
  public double getX() {
    return x;
  }


  /**
   * Returns the Odometer data Y.
   * <p>
   * 
   * @return the odometer data Y.
   */
  
  public double getY() {
    return y;
  }

  /**
   * Returns the Odometer data theta.
   * <p>
   * 
   * @return the odometer data theta.
   */
  
  public double getTheta() {
    return theta;
  }

  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for odometry.
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

  /**
   * Overwrites x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public void setX(double x) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites y. Use for odometry correction.
   * 
   * @param y the value of y
   */
  public void setY(double y) {
    lock.lock();
    isResetting = true;
    try {
      this.y = y;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites theta. Use for odometry correction.
   * 
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Method to manually set the x-axis position of the robot.
   * 
   * @param Xposition
   */
  public void setXCorr(double Xposition) {
    x = Xposition;
  }

  /**
   * Method to manually set the y-axis position of the robot.
   * 
   * @param Xposition
   */
  public void setYCorr(double Yposition) {
    y = Yposition;
  }

}


