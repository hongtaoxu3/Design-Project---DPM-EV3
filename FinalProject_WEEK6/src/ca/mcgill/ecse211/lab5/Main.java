package ca.mcgill.ecse211.lab5;


import static ca.mcgill.ecse211.lab5.Resources.*;


/**
 * This class combines all other classes together to perform the sequence of tasks
 * required to complete the final project.
 * 
 * @author Tony
 *
 */

public class Main {

  public static void main(String[] args) throws InterruptedException {

    // Starts odometer thread and display (testing) thread
    Navigation nav = new Navigation();

    new Thread(ultrasonicPoller).start();
    //      OdometerCorrection correction = new OdometerCorrection(nav, odometer);

    new Thread(odometer).start();
    new Thread(new Display()).start();

    //      odometer.setXYT(TILE_SIZE * 14, TILE_SIZE, 270.0);
    //      nav.travelTo(10.5, 2.0);

    //
    //UltrasonicLocalizer USlocalizer = new UltrasonicLocalizer();
    //USlocalizer.localize();
    //      System.out.println("greenTeam: "+ greenTeam);
    //      System.out.println("Teamnumber: "+ TEAM_NUMBER);
    //
    LightLocalizer lightLocalizer = new LightLocalizer();
    //odometer.setXYT(TILE_SIZE, TILE_SIZE * 8, 90.0);
    //      //lightLocalizer.traveltoTunnel();
    lightLocalizer.localize(); //calls traveltotunnel method which drives robot to front of tunnel
    //
    ////      correction correction = new correction();
    ////      correction.correct();
    //      
    //      //correction before tunnel
    OdometerCorrection odoCorrection = new OdometerCorrection(nav, odometer);
    odoCorrection.CorrectAngle();
    //      
    //      //travel through tunnel
    lightLocalizer.travelthrough();

    //correction after tunnel
    //OdometerCorrection odoCorrection2 = new OdometerCorrection(nav, odometer);
    //odoCorrection2.CorrectAngle();
    //      
    //      
    //      //avoidance and launch
    //odometer.setXYT(TILE_SIZE * 14, TILE_SIZE, 270.0);
    Launcher launcher = new Launcher();
    launcher.travelAndLaunch();
    //      
    //      //travel to tunnel , odo correct, pass tunnel, odo correct
    lightLocalizer.goBackfront();
    //      
    odoCorrection.CorrectAngle();
    //      
    lightLocalizer.travelthrough();
    //      
    //      odoCorrection.CorrectAngle();
    //      
    //      //go back to corner
    lightLocalizer.backToCorner();


    leftMotor.stop(true);
    rightMotor.stop();

  }
}
