package ca.mcgill.ecse211.lab3;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid 
 * cluttering the rest of the codebase. All resources can be imported at once like this:
 * 
 * <p>{@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */
public class Resources {

  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.130;

  /**
   * The robot width in centimeters.
   */
  public static final double TRACK = 15.15;

  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 250;

  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 150;
  
  /**
   * The speed at which the US sensor turns.
   */
  public static final int SCAN_SPEED = 100;

  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 3000;
  
  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int MOTOR_LOW = 100;
  
  /**
   * Speed of the faster rotating wheel (deg/sec).
   */
  public static final int MOTOR_HIGH = 200;

  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD = 3000;

  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.48;

  /**
   * Filter time (ms)
   */

  public static final int FILTER_OUT = 20;

  /**
   * Offset from the wall (cm).
   */
  public static final int BAND_CENTER = 15;

  /**
   * Width of dead band (cm).
   */
  public static final int BAND_WIDTH = 3;

  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int OBSTACLE_LOW = 135;

  /**
   * Speed of the faster rotating wheel (deg/sec).
   */
  public static final int OBSTACLE_HIGH = 200;
  
  /**
   * The turning in speed while in obstacle avoidance mode.
   */
  public static final int OBSTACLE_TURN_IN_SPEED = 225;
  
  /**
   * The obstacle turn out speed while in obstacle avoidance mode.
   */
  public static final int OBSTACLE_TURN_OUT_SPEED = 25;
  
  /**
   * The sensor angle while bang-bang style controller is engaged.
   */
  public static final int OBSTACLE_SENSOR_ANGLE = -50;

  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor US_SENSOR = 
      new EV3UltrasonicSensor(LocalEV3.get().getPort("S3"));

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * The sensor motor.
   */
  public static final EV3LargeRegulatedMotor sensorMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  
  /**
   * Maximum right angle of the US sensor
   */
  public static final int SENSOR_RIGHT_ANGLE = 55;
  
  /**
   * Maximum left angle of the US sensor
   */
  public static final int SENSOR_LEFT_ANGLE = -55;

  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();

}
