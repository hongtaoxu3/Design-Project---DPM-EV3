package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;

import static ca.mcgill.ecse211.lab5.Resources.*;

public class Main {
    
    public static void main(String[] args) throws InterruptedException {
        int buttonChoice;
        leftLauncher.lock(15);
        rightLauncher.lock(15);
        do {
            //Clear the display
            LCD.clear();
            //Prompt the user for falling edge or rising edge
            LCD.drawString("< Left | Right >", 0, 0);
            LCD.drawString(" Travel| Stand  ", 0, 1);
            LCD.drawString(" to    | and    ", 0, 2);
            LCD.drawString(" and   | Shoot  ", 0, 3);
            LCD.drawString(" Shoot |        ", 0, 4);
    
            buttonChoice = Button.waitForAnyPress();//Wait for the user to decide

        } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
        

        if (buttonChoice == Button.ID_LEFT) {
            new Thread(odometer).start(); // start odo thread
            new Thread(new Display()).start(); //start display thread
            new Thread(new UltrasonicLocalizer()).start(); //start ultrasonic localizer thread
        }
        if (buttonChoice == Button.ID_RIGHT) {
//        	Launcher.launchPosition();
//        	Launcher.launchBall();
        	Thread.sleep(1000);
        	Launcher.launchPosition();
        	LCD.clear();
            LCD.drawString(" Press Right to ", 0, 2);
            LCD.drawString("       Shoot    ", 0, 3);
        }
        
       
        while (Button.waitForAnyPress() == Button.ID_RIGHT) {
            Launcher.launchBall();
    
            Thread.sleep(400);
            Launcher.launchPosition();
        }
        
        if (Button.waitForAnyPress() == Button.ID_ESCAPE) {
            System.exit(0);
        }
    }
}
