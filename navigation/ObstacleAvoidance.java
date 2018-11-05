package navigation;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.Odometer;
import odometer.OdometerExceptions;
import usSensor.UltrasonicPoller;

/**
 * This is the class for navigation with obstacle avoidance
 * THIS VERSION IS NOT FOR PREVIOUS LAB.
 * @author jealo
 *
 */
public class ObstacleAvoidance {
  private static final int FORWARD_SPEED = 200;
  private static final int ROTATE_SPEED = 100;
  private static final double TILE_SIZE = 30.48;
  // add points here
  private static final double[] x = {1 * TILE_SIZE, 0 * TILE_SIZE, 2 * TILE_SIZE, 2 * TILE_SIZE, 1 * TILE_SIZE};
  private static final double[] y = {1 * TILE_SIZE, 2 * TILE_SIZE, 2 * TILE_SIZE, 1 * TILE_SIZE, 0 * TILE_SIZE};
  // orientation values
  private static Odometer odo;
  private static double routeAngle; // the angle relative to Y-axis of the direction of the route (0, 360)
  private static double currentRoute; // to be compared with routeAngle
  private static double deltaX, deltaY;
  private static double currentX;
  private static double currentY;
  private static double currentT;
  private static double theta = 0.0; // the angle needed to turn at each time
  private static final double pi = Math.PI;
  // ultrasonic sensor measurement
  private static int distance;
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
    
	ObstacleAvoidance.leftMotor = leftMotor;
	ObstacleAvoidance.rightMotor = rightMotor;
	ObstacleAvoidance.odo = odo;
	ObstacleAvoidance.leftRadius = leftRadius;
	ObstacleAvoidance.rightRadius = rightRadius;
	ObstacleAvoidance.track = track;
	// reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(1000);
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
      
      // travel to the target point
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
	  
	  // here we are trying to find theta based on different cases,
	  // where theta is the angle that the robot needs to rotate
      currentT = odo.getXYT()[2];
      if (deltaX == 0 && deltaY > 0) {
    	  theta = - currentT;
    	  routeAngle = 0;
      }
      else if (deltaX == 0) {
    	  theta = 180 - currentT;
    	  routeAngle = 180;
      }
      else if (deltaY == 0 && deltaX < 0) {
    	  theta = - currentT - 90;
    	  routeAngle = -90;
      }
      else if (deltaY == 0) {
    	  theta = - currentT + 90;
    	  routeAngle = 90;
      }
      else {
    	  theta = - currentT + 90 - Math.atan2(deltaY , deltaX) * 180 / pi;
    	  routeAngle = 90 - Math.atan2(deltaY , deltaX) * 180 / pi;
      }
      if (theta > 180) theta -= 360;
      if (theta < -180) theta += 360;
      
      // take the turn
      turnTo(theta);
        
      // reserve the absolute angle for route recovering in the avoidance process
      if (routeAngle < 0) routeAngle += 360;      
      
      // drive forward
      runForward (i, routeAngle);
  }
  
  /**
   * This method causes the robot to turn (on point) a degree of theta. 
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
   * This method controls the forward moving of the robot, along with obstacle avoidance
   * @param toRun
   */
  private static void runForward (int i, double routeAngle) {
	  while (!arrived(i)) {
		leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED);
	    leftMotor.forward();
	    rightMotor.forward();
	    distance = UltrasonicPoller.get_distance(); // get the distance from UltrasonicPoller
	    
	    // if the distance is too small, which means obstacle exists ahead,
	    // we go to the avoidance method
		if (distance <= 12) {
			leftMotor.setSpeed(0);
			rightMotor.setSpeed(0);
			obstacleAvoidance(i, routeAngle);
		}
	  }
  }
  
  /**
   * This method indicates whether the robot has arrived the next point, within an error of 2 cm
   * @param nextX
   * @param nextY
   * @return
   */
  private static boolean arrived(int i) {
	  currentX = odo.getXYT()[0];
	  currentY = odo.getXYT()[1];
	  if (Math.sqrt((currentX -  x[i]) * (currentX - x[i]) + (currentY - y[i]) * (currentY - y[i]))
			  >= 1.99){
		  return false;
	  }
	  else return true;
  }
  
  /**
   * This method deals with obstacle and 
   * adjust the robot back to the original route and direction after avoidance
   */
  private static void obstacleAvoidance (int i, double routeAngle) {
	  do {
	      distance = UltrasonicPoller.get_distance();
	      if (distance <= 7) { // if distance is too small, take a sharp turn right
	    	  leftMotor.setSpeed(ROTATE_SPEED - 75);
	          rightMotor.setSpeed(ROTATE_SPEED);
	          leftMotor.backward();
	          rightMotor.backward();
	      }
	      else if (distance <= 15) { // if distance is accepted, go straight
	    	  leftMotor.setSpeed(ROTATE_SPEED);
	          rightMotor.setSpeed(ROTATE_SPEED);
	          leftMotor.forward();
	          rightMotor.forward();
	      }
	      else if (distance >= 30) { // if distance is too big, turn left to find the obstacle
	    	  leftMotor.setSpeed(ROTATE_SPEED - 38);
	          rightMotor.setSpeed(ROTATE_SPEED + 10);
	          leftMotor.forward();
	          rightMotor.forward();
	      }
	      else { // other case, turn left slowly
	    	  leftMotor.setSpeed(ROTATE_SPEED - 10);
	          rightMotor.setSpeed(ROTATE_SPEED + 10);
	          leftMotor.forward();
	          rightMotor.forward();
	      }
	  } while ((!avoided(i, routeAngle)));
	  
	  // adjust the angle of direction to routeAngle
	  currentT = odo.getXYT()[2];
	  while (Math.abs(currentT - routeAngle) > 1.0) {
		  if (currentT < routeAngle) {
			leftMotor.setSpeed(ROTATE_SPEED + 25);
          	rightMotor.setSpeed(ROTATE_SPEED - 25);
          	leftMotor.forward();
          	rightMotor.backward();
		  }
		  else {
				leftMotor.setSpeed(ROTATE_SPEED - 25);
	          	rightMotor.setSpeed(ROTATE_SPEED + 25);
	          	leftMotor.backward();
	          	rightMotor.forward();
		  }
		  currentT = odo.getXYT()[2];
	  }
	  
  }
  
  /**
   * This method checks if the robot is back to the original route after avoidance
   * @param i
   * @param routeAngle
   * @return
   */
  private static boolean avoided (int i, double routeAngle) {
	  currentX = odo.getXYT()[0];
	  currentY = odo.getXYT()[1];
	  currentT = odo.getXYT()[2];
	  currentRoute = Math.atan((x[i] - currentX) / (y[i] - currentY)) * 180 / pi;
	  if (currentRoute < 0) currentRoute += 360; // set currentRoute into a range of (0, 360)
	  
	  // check if back to original route
	  if ( Math.abs(currentRoute - routeAngle) < 3 ) {
		  if (currentT - currentRoute < 0 && Math.abs(currentT - currentRoute) < 180) { // ensure that it is at the end of avoidance, instead of beginning
			  return true;
		  }
	  }
	  if (arrived(i)) return true;
	  return false;
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
