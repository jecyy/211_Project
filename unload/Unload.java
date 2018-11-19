package unload;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class controls the motor that spins to unload the rings at the end of the duration.
 * 
 * @author Hooley
 *
 */
public class Unload {
	
	/**
	 * @param args
	 */
	  private static EV3LargeRegulatedMotor unloadMotor;
	  private static int unloadSpeed = 100;
	  private static int unloadAngle = 90;
	  
	  public static void unload(){
		  
		  unloadMotor.setSpeed(unloadSpeed);
		  unloadMotor.rotate(unloadAngle);
		  
		  try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	  }
	  
}
