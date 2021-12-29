package ca.mcgill.ecse211.lab5;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class Resources {
  
  /**
   * The wheel radius in centimeters. 2.14
   */
  public static final double WHEEL_RAD = 2.14; 
  
  /**
   * The robot width in centimeters. 16.4
   */
  public static final double TRACK = 16.0;
  
  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 200; //250
  
  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 75; //120
  
  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 3000;
  
  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD = 3000;
  
  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.33;
  

  
  public static final int FILTER_OUT = 20;
  
  public static final int WALL_DISTANCE = 33; 
  /**
   * Distance buffer to ensure the robot begins rotating (cm)
   */
  public static final int BUFFER = 3;
  
  public static final double SENSOR_LOCATION = 16.0;
  
  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  

 
  /**
   * The leftLauncher
   */
  public static final EV3LargeRegulatedMotor leftLauncher = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  
  
  /**
   * The leftLauncher
   */
  public static final EV3LargeRegulatedMotor rightLauncher = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();
  /*
   * The color Sensor
   */
  public static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
 /*
  * The ultrasonic Sensor
  */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
  
  
  public static Launcher launcher;
   
  //public static final int LAUNCH_ANGLE = 90;
  /**
   * Motor speed for launching projectile
   */
  public static final int LAUNCH_SPEED = 760;
  
  /**
   * Motor acceleration for launching projectile
   */
  public static final int LAUNCH_ACCELERATION = 10000;
  
/*
   * MapSize {x, y}. Assuming that starting point of our robot is (0,0)
   */
  //public static final int[] mapSize = {8, 8};
  
  /**
   * Centre of shooting point (x,y)
   */
 // public static final double[] targetLocation = {6*TILE_SIZE+15.23,2*TILE_SIZE+15.23};
  public static final double[] targetLocation = {5 * TILE_SIZE , 5*TILE_SIZE };
  public static final int RADIUS_TARGET_LOCATION = 145;
  
  
}