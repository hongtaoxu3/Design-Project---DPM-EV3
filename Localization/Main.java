package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import static ca.mcgill.ecse211.lab4.Resources.*;

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

    // Odometer thread

    new Thread(odometer).start();
    
    // Create instance of US localizer depending on button choice
    // (Rising or falling edge)

    buttonChoice = chooseClockType();

    if (buttonChoice == Button.ID_RIGHT) {
      new Thread(new Display()).start();
      UltrasonicLocalizer UltrasonicLocalizer = new UltrasonicLocalizer();
      UltrasonicLocalizer.localize(false);
    }

    else {
      new Thread(new Display()).start();
      UltrasonicLocalizer UltrasonicLocalizer = new UltrasonicLocalizer();
      UltrasonicLocalizer.localize(true);
    }
    
    // After the US localizer is complete, press button to start light localizer
    
    while (Button.waitForAnyPress() != Button.ID_DOWN);
    LightLocalizer LightLocalizer = new LightLocalizer();
    LightLocalizer.localize();
    
    // Press escape to exit after program
    
    if (Button.waitForAnyPress() == Button.ID_ESCAPE) {
      System.exit(0);
    }

  }
  
  /**
   * Displays available clock choices for US localizer
   * 
   * @return
   */

  private static int chooseClockType() {
    int buttonChoice;
    Display.showText(
        "< Left | Right >",
        "       |        ",
        "Rising | Falling",
        "  Edge |  Edge  ",
        "       |        ");

    do {
      buttonChoice = Button.waitForAnyPress(); // left or right press
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
  }
}
