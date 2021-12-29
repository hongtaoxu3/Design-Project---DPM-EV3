package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

public class BangBangController extends UltrasonicController {

  public BangBangController() {
    LEFT_MOTOR.setSpeed(MOTOR_HIGH); // Start robot moving forward
    RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  @Override
  public void processUSData(int distance) {
    filter(distance);

    // Apply corrections to turn towards the wall if too far

    if (distance > BAND_WIDTH + BAND_CENTER) {
      LEFT_MOTOR.setSpeed(MOTOR_LOW); 
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }

    // Apply corrections to turn away from the wall if too close

    else if (distance < BAND_CENTER - BAND_WIDTH) {
      LEFT_MOTOR.setSpeed(MOTOR_HIGH); 
      RIGHT_MOTOR.setSpeed(MOTOR_LOW);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }

    // Move forward if the error is within the threshold

    else {

      LEFT_MOTOR.setSpeed(MOTOR_HIGH);
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();

    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
