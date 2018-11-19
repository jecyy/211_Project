package grasping;


import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.Odometer;

/**
 * This class controls the process of grasping after the robot arrives at the rings.
 * 
 * @author Hooley
 *
 */
public class Grasp {
	
	private static EV3LargeRegulatedMotor left, right;
	private static double leftR, rightR, trac;
	private static odometer.Odometer odo;
	private static double pi = Math.PI;
	private static int ROTATE_SPEED = 100, SEARCH_SPEED = 100;
	private static double treeBase = 18.5; //Distance robot has to travel before turning
	private static double ringDist = 3.0; 
	private static double colorOffset = 1.2; //Distance between ringUs and colorSensor
	
	
	public static void grasp(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius,
			double rightRadius, double track, Odometer odometer) {
		left = leftMotor;
		right = rightMotor;
		odo = odometer;
		leftR = leftRadius;
		rightR = rightRadius;
		trac = track;
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(1000);
		}
		// prepare odometer
		try {
			odo = Odometer.getOdometer();
		} catch (odometer.OdometerExceptions e1) {
		}

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
		//Saves starting location
		double orix = odo.getXYT()[0];
		double oriy = odo.getXYT()[1];
		//Tree circulating
		turn(90);
		search();
		turn(90);
		search();
		turn(90);
		search();
		turn(90);
		search();
		double x = odo.getXYT()[0];
		double y = odo.getXYT()[1];
		//Return back to starting point
		left.rotate(convertDistance(distanceConversion(x, y, orix, oriy), 5), true);
		right.rotate(convertDistance(distanceConversion(x, y, orix, oriy), 5), false);
		
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		
	}
	private static void search(){ //Causes robot to slowly drive untill it either passes the tree or sees a ring
		//Setting search speeds
		left.setSpeed(SEARCH_SPEED);
		right.setSpeed(SEARCH_SPEED);
		left.forward();
		right.forward();
		double treeX = odo.getXYT()[0];
		double treeY = odo.getXYT()[1];
		double curx = odo.getXYT()[0];
		double cury = odo.getXYT()[1];
		//If the robot does not see the ring || has not driven far enough 
		while(ringUsSensor.UltrasonicPoller.get_distance() > ringDist || convertDistance(2.2, distanceConversion( curx, cury, treeX, treeY)) >= treeBase ){ 
			left.forward();
			right.forward();
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			}
		}//If the ring was found 
		if(ringUsSensor.UltrasonicPoller.get_distance() < ringDist){
			left.setSpeed(0);
			right.setSpeed(0);
			//Call ring found
			ringFound();
			//Travels the remaining distance
			left.setSpeed(SEARCH_SPEED);
			right.setSpeed(SEARCH_SPEED);
			left.rotate(convertDistance(distanceConversion(curx, cury, treeX, treeY), 5), true);
			right.rotate(convertDistance(distanceConversion(curx, cury, treeX, treeY), 5), false);
			
		}
		//If no ring was found sleeps thread and goes back
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}	
	}
	
	private static void ringFound() { //Method called once a ring has been found
		left.setSpeed(SEARCH_SPEED);
		right.setSpeed(SEARCH_SPEED);
		//usSensor and colo	r sensor are offset by 2.2cm
		left.rotate(convertDistance(colorOffset, 5), true);
		right.rotate(convertDistance(colorOffset, 5), false);
		//Call color sensor
		int color = findMatch();
		//Return beeps accordingly
		if(color == 0){ //blue
			Sound.beep(); 
		}else if(color == 1){ //green
			Sound.twoBeeps();
		}else if(color == 2){ //yellow
			Sound.beep();
			Sound.twoBeeps();
		}else{ //Orange	
			Sound.twoBeeps();
			Sound.twoBeeps();
		}
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
	}
	private static int findMatch() {
		double r = lightSensor.ColorSensorPoller.getR(), g = lightSensor.ColorSensorPoller.getG(), b = lightSensor.ColorSensorPoller.getB();
		if (r < b) // blue
			return 0;
		else if (r < g) // green
			return 1;
		else if (r > 2 * g)
			return 3; // orange
		else
			return 2; // yellow

		
	}
	private static void turn(double theta) { //Simple turning method
		left.setSpeed(ROTATE_SPEED);
		right.setSpeed(ROTATE_SPEED);
		left.rotate(convertAngle(leftR, trac, theta), true);
		right.rotate(-convertAngle(rightR, trac, theta), false);
	}
	private static double distanceConversion(double curx, double cury, double treeX, double treeY){
		return Math.sqrt((curx - treeX) * (curx - treeX) + (cury - treeY) * (cury - treeY));
	}
	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (pi * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, pi * width * angle / 360.0);
	}
}
