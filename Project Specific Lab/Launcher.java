package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Resources.*;

public class Launcher {

	/*
	 * the launcher pull back
	 */
	
  public static void launchPosition() {
       rightLauncher.setSpeed(500);
       leftLauncher.setSpeed(500);
      rightLauncher.setSpeed(ROTATE_SPEED);
      leftLauncher.setSpeed(ROTATE_SPEED);
      
      rightLauncher.rotate(-130, true);
      leftLauncher.rotate(-130, false);
      rightLauncher.flt();
      leftLauncher.flt();
  }
  
  /**
   * launch the ball
   */
  public static void launchBall() {
      rightLauncher.setSpeed(LAUNCH_SPEED);
      leftLauncher.setSpeed(LAUNCH_SPEED);
      
      rightLauncher.setAcceleration(LAUNCH_ACCELERATION);
      leftLauncher.setAcceleration(LAUNCH_ACCELERATION);
      
      leftLauncher.rotate(130, true);
      rightLauncher.rotate(130, false);
      
      rightLauncher.stop();
      leftLauncher.stop();
  }
  
  /**
   * descends the launcher arm
//   */
//  public static void reloadBall() {
//      rightLauncher.setSpeed(ROTATE_SPEED);
//      leftLauncher.setSpeed(ROTATE_SPEED);
//      
//      rightLauncher.rotate(LAUNCH_ANGLE, true);
//      leftLauncher.rotate(-LAUNCH_ANGLE, false);
//  }
}
