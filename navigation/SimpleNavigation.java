package navigation;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.Odometer;
import odometer.OdometerExceptions;

/**
 * This class is for the simple navigation
 * THIS VERSION IS FOR PREVIOUS LAB.
 * @author jealo
 *
 */
public class SimpleNavigation {
  private static final int FORWARD_SPEED = 200;
  private static final int ROTATE_SPEED = 100;
  private static final double TILE_SIZE = 30.48;
  // add points here
  private static final double[] x = {1 * TILE_SIZE, 0 * TILE_SIZE, 2 * TILE_SIZE, 2 * TILE_SIZE, 1 * TILE_SIZE};
  private static final double[] y = {1 * TILE_SIZE, 2 * TILE_SIZE, 2 * TILE_SIZE, 1 * TILE_SIZE, 0 * TILE_SIZE};
  // orientation values
  private static Odometer odo;
  private static double deltaX, deltaY;
  private static double toTravel;
  private static double currentT;
  private static double theta = 0.0; // the angle needed to turn at each time
  private static final double pi = Math.PI;
  // the motors
  private static EV3LargeRegulatedMotor leftMotor;
  private static EV3LargeRegulatedMotor rightMotor;
  private static double leftRadius, rightRadius, track;

  /**
   * This method is meant to drive the robot according to the assigned points. It is to run in parallel
   * with the Odometer and UltrasonicPoller classes allow testing their functionality.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param width
   */
  public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double leftRadius, double rightRadius, double track, Odometer odo) {
    
	SimpleNavigation.leftMotor = leftMotor;
	SimpleNavigation.rightMotor = rightMotor;
	SimpleNavigation.odo = odo;
	SimpleNavigation.leftRadius = leftRadius;
	SimpleNavigation.rightRadius = rightRadius;
	SimpleNavigation.track = track;
	// reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(300);
    }
    
    // prepare odometer
	try {
		odo = Odometer.getOdometer();
	} catch (OdometerExceptions e1) {
	}

    // Sleep for 2 seconds
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
    // initialize x and y
    deltaX = x[0] - 0;
    deltaY = y[0] - 0;

    for (int i = 0; i < 5; i++) {
    	
      travelTo(i);
      
      // update deltaX and deltaY
      if (i < 4) {
    	  deltaX = x[i + 1] - x[i];
    	  deltaY = y[i + 1] - y[i];
      }
    }
    
    leftMotor.stop();
    rightMotor.stop();
  }
  
  /**
   * This method causes the robot to travel to the absolute field location (x, y), 
   * specified in tile points. 
   * This method should continuously call turnTo(double theta) 
   * and then set the motor speed to forward(straight). 
   * This will make sure that your heading is updated until you reach your exact goal. 
   * This method will poll the odometer for information. 
   * @param x
   * @param y
   */
  private static void travelTo(int i) {
      toTravel = Math.sqrt(deltaX * deltaX + deltaY * deltaY); // calculate the distance between the current and next way point
      
	  // here we are trying to find theta based on different cases,
	  // where theta is the angle that the robot needs to rotate
      currentT = odo.getXYT()[2];
      if (deltaX == 0 && deltaY >= 0) {
    	  theta = - currentT;
      }
      else if (deltaX == 0) {
    	  theta = 180 - currentT;
      }
      else if (deltaY == 0 && deltaX < 0) {
    	  theta = - currentT - 90;
      }
      else if (deltaY == 0 && deltaX > 0) {
    	  theta = - currentT + 90;
      }
      else {
    	  theta = 90 - Math.atan(deltaY / deltaX) * 180 / pi - currentT;
    	  if (deltaX <= 0 && deltaY <= 0) {
    		  theta = theta - 180;
    	  }
    	  if (deltaX <= 0 && deltaY >= 0) {
    		  theta = - theta;
    	  }
      }
      if (theta >= 180) theta -= 360;
      if (theta <= -180) theta += 360;
      
      // take the turn
      turnTo(theta);
        
      // drive forward
      
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.rotate(convertDistance(leftRadius, toTravel), true);
      rightMotor.rotate(convertDistance(rightRadius, toTravel), false);
  }
  
  /**
   * This method causes the robot to turn (on point) to the absolute heading theta. 
   * This method should turn a MINIMAL angle to its target. 
   * @param theta
   */
  private static void turnTo (double theta) {
	  leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
        
      leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
      rightMotor.rotate(-convertAngle(rightRadius, track, theta), false);
  }
  
  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}

