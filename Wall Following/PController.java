package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.*;

public class PController extends UltrasonicController {

  private static final int MOTOR_SPEED = 200;

  // Added constants (Maximum speed and gain constant)

  private final int MAX_SPEED = 175;

  private static final double GAIN_CONSTANT = 8;

  public PController() {
    LEFT_MOTOR.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  // Function to calculate the speed correction using the gain constant

  private float calculateSpeed(float errorValue) {

    int speedCorrection;

    if (errorValue < 0) {
      errorValue = -errorValue;
    }

    // As seen in the slides, the correction is the distance error times the gain constant

    speedCorrection = (int) (GAIN_CONSTANT * errorValue);

    // Set a maximum speed 

    if (speedCorrection > MAX_SPEED) {
      speedCorrection = MAX_SPEED;
    }

    return speedCorrection;

  }

  @Override
  public void processUSData(int distance) {
    filter(distance);

    // Distance error

    int error = BAND_CENTER - distance;

    // Initialize variables (left/right motor speeds, corrections)

    float speed_correction;
    float left_speed; 
    float right_speed;

    // Move forward if the error is within the threshold

    if (Math.abs(error) <= BAND_WIDTH) {
      LEFT_MOTOR.setSpeed(MOTOR_HIGH); 
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }

    // Apply corrections to turn away from the wall if too close

    else if (error > 0) {

      // Increase turn efficiency if way too close of wall

      if (distance < 10) {
        speed_correction = calculateSpeed(error);
        left_speed = MOTOR_SPEED - speed_correction/5;
        right_speed = MOTOR_SPEED - speed_correction/5;
        LEFT_MOTOR.setSpeed(left_speed); 
        RIGHT_MOTOR.setSpeed(right_speed);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.backward();
      }

      // Normal correction

      else {
        speed_correction = calculateSpeed(error);
        left_speed = MOTOR_SPEED + speed_correction * 11/10;
        right_speed = MOTOR_SPEED - speed_correction/5;
        LEFT_MOTOR.setSpeed(left_speed); 
        RIGHT_MOTOR.setSpeed(right_speed);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
      }
    }

    // Apply corrections to turn towards the wall if too far

    else if (error < 0) {
      speed_correction = calculateSpeed(error);
      right_speed = MOTOR_SPEED + speed_correction * 4/6;
      left_speed = MOTOR_SPEED - speed_correction/5;
      LEFT_MOTOR.setSpeed(left_speed); 
      RIGHT_MOTOR.setSpeed(right_speed);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }

    // Move away from wall if weird values

    else if (error < -500) {
      speed_correction = calculateSpeed(error);
      right_speed = MOTOR_SPEED - speed_correction/5;
      left_speed = MOTOR_SPEED - speed_correction/5;
      LEFT_MOTOR.setSpeed(left_speed); 
      RIGHT_MOTOR.setSpeed(right_speed);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }

  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}