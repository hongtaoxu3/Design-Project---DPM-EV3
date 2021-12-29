// Lab2.java
package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;

// static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab3.Resources.*;

/**
 * The main driver class for the odometry lab.
 */
public class Main {

  /**
   * The main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) {
    int buttonChoice;
    new Thread(odometer).start();

    buttonChoice = chooseObstaclesOrNot();

    if (buttonChoice == Button.ID_RIGHT) {
      new Thread(new NavigationWithObstacles()).start();
    } else {
      new Thread(new Navigation()).start();
    }

    new Thread(new Display()).start();
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    } // do nothing

    System.exit(0);
  }

  private static int chooseObstaclesOrNot() {
    int buttonChoice;
    Display.showText("< Left | Right >",
                     "       |        ",
                     "  No   |  Yes   ",
                     "  Obs  |  Obs   ",
                     "       |        ");
    
    do {
      buttonChoice = Button.waitForAnyPress(); // left or right press
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
  }

  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration sleep duration in milliseconds
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }

}
