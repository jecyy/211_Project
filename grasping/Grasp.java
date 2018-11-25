package grasping;


import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lightSensor.LightSensorLeft;
import lightSensor.LightSensorRight;
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
	private static int ROTATE_SPEED = 100, SEARCH_SPEED = 100, TRAVEL_SPEED = 200;
//	private static double treeBase = 20.0; //Distance robot has to travel before turning
	private static double ringDist = 3.0; 
	private static double offset = 10.0; // distance from light sensor to the wheel
	public static final double WHEEL_RAD = 2.2;
	private static final double ts = 30.48;
	private static final int black = 100;
	private static final int DETECTION_PERIOD = 30;
	
	
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
		//Travel to center of square
		adjustOrientation();
		left.setSpeed(TRAVEL_SPEED);
		right.setSpeed(TRAVEL_SPEED);
		left.rotate(convertDistance(WHEEL_RAD, ts/2 - offset), true);
		right.rotate(convertDistance(WHEEL_RAD, ts/2 - offset), false);
		turn(90);
		adjustOrientation();
		left.rotate(-convertDistance(WHEEL_RAD, ts/2 + offset), true);
		right.rotate(-convertDistance(WHEEL_RAD, ts/2 + offset), false);
		//turn(-90);
		//Tree circulating
		search();
		turn(-90);
		search();
		turn(-90);
		search();
		turn(-90);
		search();
		double x = odo.getXYT()[0];
		double y = odo.getXYT()[1];
		//Return back to starting point TODO: change this to drive until sees line
		left.rotate(convertDistance(distanceConversion(x, y, orix, oriy), 5), true);
		right.rotate(convertDistance(distanceConversion(x, y, orix, oriy), 5), false);
		
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		
	}
	private static void search(){ //Causes robot to slowly drive until it either passes the tree or sees a ring
		//Setting search speeds
		left.setSpeed(SEARCH_SPEED);
		right.setSpeed(SEARCH_SPEED);

		//If robot is not at right spot to read 
		left.rotate(-convertDistance(WHEEL_RAD, ts/2), true);
		right.rotate(-convertDistance(WHEEL_RAD, ts/2), false);
		//Ring is in front of sensor
		left.setSpeed(0);
		right.setSpeed(0);
		//Call ring found
		ringFound();
		//Travels the remaining distance
		left.setSpeed(SEARCH_SPEED);
		right.setSpeed(SEARCH_SPEED);
		left.rotate(-convertDistance(WHEEL_RAD, ts/2 + ringDist), true);
		right.rotate(-convertDistance(WHEEL_RAD, ts/2 + ringDist), false);
			
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}	
	}
	
	private static void ringFound() { //Method called once a ring has been found
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
		if (theta > 180) theta -= 360;
		if (theta < -180) theta += 360;
		//if (theta > 0) theta += 20;
		left.setSpeed(ROTATE_SPEED);
		right.setSpeed(ROTATE_SPEED);
		left.rotate(-convertAngle(leftR, trac, theta), true);
		right.rotate(convertAngle(rightR, trac, theta), false);
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
	
	/**
	 * This method travels the robot backward until both motors see the line,
	 * making sure that the two motors start at the same line.
	 */
	private static void adjustOrientation() {
		double leftlight1 = 0, rightlight1 = 0, leftlight2 = 0, rightlight2 = 0; // 2 : current reading; 1: previous reading
		boolean left_passed = false, right_passed = false; // indicates that the left/right light sensor has passed a line
		// go backward
		left.setSpeed(TRAVEL_SPEED);
		right.setSpeed(TRAVEL_SPEED);
		left.forward();
		right.forward();
		// stop at the line
		while(true) { // if loop breaks when both sensors have seen the line
			leftlight2 = LightSensorLeft.get_light();
			rightlight2 = LightSensorRight.get_light();
			if (leftlight1 - leftlight2 > black) {
				freeze();
				left_passed = true;
			    //left_time = System.currentTimeMillis();
			    Sound.beep();
			    break;
			}
			if (rightlight1 - rightlight2 > black) {
				freeze();
				right_passed = true;
			    //right_time = System.currentTimeMillis();
			    Sound.beep();
			    break;
			}
			try {
				Thread.sleep(DETECTION_PERIOD);
			} catch (InterruptedException e) {

			}
			leftlight1 = leftlight2;
			rightlight1 = rightlight2;
		}
		freeze();
		
		if (left_passed) {
			right.setSpeed(TRAVEL_SPEED);
			right.forward();
			while(true) {
				rightlight2 = LightSensorRight.get_light();
				if (rightlight1 - rightlight2 > black) break; 
				try {
					Thread.sleep(DETECTION_PERIOD);
				} catch (InterruptedException e) {

				}
				rightlight1 = rightlight2;
			}
		}
		else { // right passed
			left.setSpeed(TRAVEL_SPEED);
			left.forward();
			while(true) {
				leftlight2 = LightSensorLeft.get_light();
				if (leftlight1 - leftlight2 > black) break; 
				try {
					Thread.sleep(DETECTION_PERIOD);
				} catch (InterruptedException e) {

				}
				leftlight1 = leftlight2;
			}
		}
		freeze();
		Sound.beep();
	}
	
	/**
	 * This method sets the speeds of both left and right motors to 0
	 */
	private static void freeze() {
		left.setSpeed(0);
		right.setSpeed(0);
	}
}
