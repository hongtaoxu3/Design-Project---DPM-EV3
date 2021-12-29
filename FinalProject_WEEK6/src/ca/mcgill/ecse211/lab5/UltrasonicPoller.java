package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Resources.*;

/**
 * This class provides method to poll the current value read by the ultrasonic sensor.
 * It is a thread.
 * 
 * @author Xinran Li
 *
 */

public class UltrasonicPoller implements Runnable{
  public float[] usData;
  int distance;
  private static UltrasonicPoller poller;

  /**
   * Default constructor, initializes an array to store the data
   * 
   */

  private UltrasonicPoller() {
    usData = new float[usSensor.sampleSize()];
  }

  /**
   * Method that gets the current distance read by the ultrasonic sensor every 15 ms.
   * The distance is multiplied by 100 for better readability.
   * 
   */

  public void run() {
    while(true) {
      usSensor.getDistanceMode().fetchSample(usData, 0);
      distance = (int) (usData[0]*100.0);
      //System.out.println("usData: " + distance);
      try {
        Thread.sleep(15);
      } catch (Exception e) {
        // TODO: handle exception
      }
    }
  }

  /**
   * Method that returns the current read distance.
   *  
   * @return The distance read by the ultrasonic sensor
   */

  public int getDistance() {
    return this.distance;
  }

  /**
   * Method that returns an instance of a ultrasonic poller.
   * 
   * @return An instance of a ultrasonic poller
   */

  public synchronized static UltrasonicPoller getPoller() {
    if (poller == null) {
      poller = new UltrasonicPoller();
    }
    return poller;
  }
}



