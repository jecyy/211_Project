package navigation;

import grasping.Grasp;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lightSensor.LightSensorLeft;
import lightSensor.LightSensorRight;
import odometer.Odometer;
import unload.Unload;

/**
 * This class implements the traveling algorithm,
 * the robot will only run along the X-axis and Y-axis directions
 * so that the odometer is corrected every time the robot passes a tile.
 * @authors jecyy, PaulHooley
 *
 */
public class Navigation {
	/*TODO: Fix odometer, it dirves backwards we have a few different solutions
	 * 		1. Use heading variable that determines where it should be facing and how it should turn
	 *		2. Actually fix odometer 
	 *		3. Change orientation of motors
	 */
	public static int[] leftaxis = new int[2];  // TODO: be consistent with starting position 
	public static int[] rightaxis = new int[2]; // grid position of the light sensors
	private static EV3LargeRegulatedMotor leftM;
	private static EV3LargeRegulatedMotor rightM;
	private static double radius, trac;
	private static Odometer odo;
	private static double lightTrac = 7.5; // TODO: measure the distance between two light sensors
	private static double pi = Math.PI;
	private static final int ROTATE_SPEED = 75;
	private static final int FORWARD_SPEED = 175;
	private static final double ts = 30.48;
	private static final int black = 100; // threshold for black line
	private static final double offset = 10; // TODO: the distance between the center of light-track and the center of rotation
	private static int currentx, currenty;
	private static final int DETECTION_PERIOD = 30;

	/**
	 * This method controls the navigation process and is the one that called in the main thread
	 * @param leftMotor
	 * @param rightMotor
	 * @param leftRadius
	 * @param rightRadius
	 * @param track
	 * @param odometer
	 */
	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double Radius, double track, Odometer odometer, 
			int tn_ll_x, int tn_ll_y, int tn_ur_x, int tn_ur_y, int starting_corner, int Tx, int Ty, int islandury, int islandlly) {
		leftM = leftMotor;
		rightM = rightMotor;
		radius = Radius;
		trac = track;
		odo = odometer;
		
		
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftM, rightM}) {
			motor.stop();
			motor.setAcceleration(6000);
		}
		
		
		final int T_Length = tunnelLength(tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y);
		final boolean tunnelIsVertical = tunnelIsVertical( tn_ll_y, tn_ur_y, starting_corner, islandury, islandlly);
		final int startx, starty;
		final boolean toLL = toLL(tn_ll_x, tn_ll_y, tn_ur_x, tn_ur_y, starting_corner, tunnelIsVertical);
		// adjust startx, starty, currentx, currenty first
		if (starting_corner == 0) {
			startx = 1;
			starty = 1;
			currentx = 1;
			currenty = 1;
			odo.setTheta(0);
		}
		else if (starting_corner == 1) {
			startx = 7;
			starty = 1;
			currentx = 7;
			currenty = 1;
			odo.setTheta(270);
		}
		else if (starting_corner == 2) {
			startx = 7;
			starty = 7;
			currentx = 7;
			currenty = 7;
			odo.setTheta(180);
		}
		else { // starting_corner == 3
			startx = 1;
			starty = 7;
			currentx = 1;
			currenty = 7;
			odo.setTheta(90);
		}
		// in the very beginning, the robot should be located around the center of the starting corner
		// so we need to travel to the tunnel first
		int tunnelx , tunnely; // the tile we need to get to before entering the tunnel
		
		if (!tunnelIsVertical) { // tunnel is along x-axis
			if (toLL) {
				tunnelx = tn_ll_x - 1;
				tunnely = tn_ll_y + 1;
			}
			else {
				tunnelx = tn_ur_x + 1;
				tunnely = tn_ur_y - 1;
			}
		}
		else { // tunnel is along y-axis
			if (toLL) {
				tunnelx = tn_ll_x + 1;
				tunnely = tn_ll_y - 1;
			}
			else {
				tunnelx = tn_ur_x - 1;
				tunnely = tn_ur_y + 1;
			}
		}
		
		// travel to the tunnel
		travelTo(tunnelx, tunnely); // travel to the grid diagonal to the LL point of tunnel
		// adjust the position
		adjustOrientation();
		// now we have to head to the direction of the tunnel
		if (!tunnelIsVertical) {
			travelDistanceBack(ts / 2 - 11);
			turn(90 - odo.getXYT()[2]);
			odo.setTheta(90);
		}
		else {
			travelDistanceBack(ts / 2 - 11);
			turn(0 - odo.getXYT()[2]);
			odo.setTheta(0);
		}
		// adjust the heading
		adjustOrientation();
		// now the robot should be heading to the tunnel
		// Travel the distance of the tunnel + 1, so that we are on the other side of the tunnel
		travelDistance((T_Length + 2) * ts + offset + 5);
		// update leftaxis and rightaxis
		if (!tunnelIsVertical) { // tunnel is along x-axis
			turn(180 - odo.getXYT()[2]);
			odo.setTheta(180);
			travelDistance(ts / 2);
			if (toLL) {
				currentx = tn_ur_x + 1;
				currenty = tn_ur_y - 1;
			}
			else {
				currentx = tn_ll_x - 1;
				currenty = tn_ll_y + 1;
			}
		}
		else { // tunnel is along y-axis
			turn(270 - odo.getXYT()[2]);
			odo.setTheta(270);
			travelDistance(ts / 2);
			if (toLL) {
				currentx = tn_ur_x - 1;
				currenty = tn_ur_y + 1;
			}
			else {
				currentx = tn_ll_x + 1;
				currenty = tn_ll_y - 1;
			}
		}
		// now the robot should be positioned at the exit of the tunnel
		// (the grid diagonal to the UR point of tunnel)
		// the next step is to travel the robot to the tree
		int treex = Tx - 1, treey = Ty - 1;
		travelTo(treex, treey);
		Grasp.grasp(leftM, rightM, radius, radius, trac, odo);
		// after the grasping, the robot should travel back to the starting position
		//Returning
		// first, let's go back to the tunnel
		if (!tunnelIsVertical) { // tunnel is along x-axis
			if (toLL) {
				tunnelx = tn_ur_x + 1;
				tunnely = tn_ur_y - 1;
			}
			else {
				tunnelx = tn_ll_x - 1;
				tunnely = tn_ll_y + 1;
			}
		}
		else { // tunnel is along y-axis
			if (toLL) {
				tunnelx = tn_ur_x - 1;
				tunnely = tn_ur_y + 1;
			}
			else {
				tunnelx = tn_ll_x + 1;
				tunnely = tn_ll_y - 1;
			}
		}
		travelTo(tunnelx, tunnely);
		// adjust the position
		adjustOrientation();
		// head to tunnel
		if (!tunnelIsVertical) { // tunnel along x-axis
			travelDistanceBack(ts / 2 - 10);
			turn(270 - odo.getXYT()[2]);
			odo.setTheta(270);
		}
		else {
			travelDistanceBack(ts / 2 - 10);
			turn(180 - odo.getXYT()[2]);
			odo.setTheta(180);
		}
		// adjust heading
		adjustOrientation();
		// go thorugh the tunnel
		travelDistance((T_Length + 2) * ts + offset);
		// update odometer
		if (!tunnelIsVertical) { // tunnel is along x-axis
			turn(0 - odo.getXYT()[2]);
			odo.setTheta(0);
			travelDistance(ts / 2);
			if (toLL) {
				currentx = tn_ll_x - 1;
				currenty = tn_ll_y + 1;
			}
			else {
				currentx = tn_ur_x + 1;
				currenty = tn_ur_y - 1;
			}
		}
		else { // tunnel is along y-axis
			turn(90 - odo.getXYT()[2]);
			odo.setTheta(90);
			travelDistance(ts / 2);
			if (toLL) {
				currentx = tn_ll_x + 1;
				currenty = tn_ll_y - 1;
			}
			else {
				currentx = tn_ur_x - 1;
				currenty = tn_ur_y + 1;
			}
		}
		// now the robot should be positioned at the entrance of the tunnel with leftaxis and rightaxis updated
		// next go back to the starting corner
		travelTo(startx, starty);
		//Unload the rings
		Unload.unload();
	}

	/**
	 * This method travels the robot to the center of the tile located at (x, y),
	 * where the point (x, y) should be the lower left vertex of the tile.
	 * @param x
	 * @param y
	 */
	private static void travelTo(int x, int y) {
		int xdis = x - currentx, ydis = y - currenty;
		// x direction
		if (xdis >= 0) {
			turn(90 - odo.getXYT()[2]); // turn to 90 degree
			odo.setTheta(90);
		}
		else {
			turn(270 - odo.getXYT()[2]); // turn to 270 degree
			odo.setTheta(270);
		}
		lineTravel(Math.abs(xdis));
		currentx = x;
		// y direction
		if (ydis > 0) {
			turn(0 - odo.getXYT()[2]); // turn to 0 degree
			odo.setTheta(0);
		}
		else {
			turn(180 - odo.getXYT()[2]); // turn to 180 degree
			odo.setTheta(180);
		}
		lineTravel(Math.abs(ydis));
		currenty = y;
	}
	
	/**
	 * This method travels the robot for a given distance forward, without any correction.
	 * @param s
	 */
	private static void travelDistance(double s) {
		leftM.setSpeed(FORWARD_SPEED);
		rightM.setSpeed(FORWARD_SPEED);
		leftM.rotate(-convertDistance(radius, s), true);
		rightM.rotate(-convertDistance(radius, s), false);
	}
	
	/**
	 * This method travels the robot for a given distance backward, without any correction.
	 * @param s
	 */
	private static void travelDistanceBack(double s) {
		leftM.setSpeed(FORWARD_SPEED);
		rightM.setSpeed(FORWARD_SPEED);
		leftM.rotate(convertDistance(radius, s), true);
		rightM.rotate(convertDistance(radius, s), false);
	}
	
	/**
	 * This method lets the robot travel n tiles in its current heading direction,
	 * with odometer correction
	 * @param n
	 */
	private static void lineTravel(int n) {
		for (int i = 0; i < n; i ++) {
			travel1();
		}
	}
	
	/**
	 * This method lets the robot pass one line
	 * so it travels from its current tile to the one in front of it
	 * with odometer correction.
	 */
	private static void travel1() {
	    //long left_time = 0, right_time = 0;
		double leftlight1 = 0, rightlight1 = 0, leftlight2 = 0, rightlight2 = 0; // 2 : current reading; 1: previous reading
		go();
		boolean left_passed = false, right_passed = false; // indicates that the left/right light sensor has passed a line
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
			keepTurning(false);
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
			keepTurning(true);
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
		Sound.beep();
		
//		long dif = left_time - right_time; // time difference between the two detections
//		double d = FORWARD_SPEED * (dif / 1000) * (pi / 180) * radius; // distance traveled by the robot within dif
//		adjustOrientation(d); // adjust the orientation
		freeze();
		// correct the odometer orientation &
		// correct the odometer position
		// since we've recorded the estimated position of the two light sensors,
		// we can use them to calculate the position of center of rotation using an offset
		if (heading() == 0) {
			odo.setTheta(0);
			//odo.setY((leftaxis[1] + rightaxis[1])/2 + offset);
		}
		else if (heading() == 1) {
			odo.setTheta(90);
			//odo.setX((leftaxis[0] + rightaxis[0])/2 + offset);
		}
		else if(heading() == 2) {
			odo.setTheta(180);
			//odo.setY((leftaxis[1] + rightaxis[1])/2 - offset);
		}
		else {
			odo.setTheta(270);
			//odo.setX((leftaxis[0] + rightaxis[0])/2 - offset);
		}
		travelDistance(offset); // go beyond the line to avoid mis-detecting
	}

	/**
	 * This method travels the robot backward until both motors see the line,
	 * making sure that the two motors start at the same line.
	 */
	private static void adjustOrientation() {
		double leftlight1 = 0, rightlight1 = 0, leftlight2 = 0, rightlight2 = 0; // 2 : current reading; 1: previous reading
		boolean left_passed = false, right_passed = false; // indicates that the left/right light sensor has passed a line
		// go backward
		leftM.setSpeed(FORWARD_SPEED);
		rightM.setSpeed(FORWARD_SPEED);
		leftM.forward();
		rightM.forward();
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
			rightM.setSpeed(FORWARD_SPEED);
			rightM.forward();
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
			leftM.setSpeed(FORWARD_SPEED);
			leftM.forward();
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
	 * The robot will turn a given angle when this method is called
	 * @param theta
	 */
	private static void turn(double theta) {
		if (theta > 180) theta -= 360;
		if (theta < -180) theta += 360;
		leftM.setSpeed(ROTATE_SPEED);
		rightM.setSpeed(ROTATE_SPEED);
		leftM.rotate(-convertAngle(radius, trac, theta), true);
		rightM.rotate(convertAngle(radius, trac, theta), false);
	}

	/**
	 * Given the radius of the motor and the distance the robot needs to travel,
	 * this method returns the angle in degree that a motor should rotate
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (pi * radius));
	}

	/**
	 * This method returns the angle a motor should rotate in order for the robot to turn a given angle
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, pi * width * angle / 360.0);
	}
	
	/**
	 * This method sets the speeds of both left and right motors to 0
	 */
	private static void freeze() {
		leftM.setSpeed(0);
		rightM.setSpeed(0);
	}
	
	/**
	 * The robot runs forward with a constant speed.
	 */
	private static void go() {
		leftM.setSpeed(FORWARD_SPEED);
		rightM.setSpeed(FORWARD_SPEED);
		leftM.backward();
		rightM.backward();
	}
	
	/**
	 * This method keeps the robot rotating with a choice of clockwise or anti
	 * (this is different from the one in Localizer in that only one motor moves here)
	 * @param isClockwise
	 */
	private static void keepTurning (boolean isClockwise) {
		leftM.setSpeed(ROTATE_SPEED);
		rightM.setSpeed(ROTATE_SPEED);

		if (isClockwise == true) {
			leftM.backward();
			//rightM.forward();
		}
		else {
			//leftM.forward();
			rightM.backward();
		}
	}
	
	/**
	 * This method returns the expected heading of the robot currently.
	 * (0 for 0 degree, 1 for 90 degree, 2 for 180 degree, 3 for 270 degree)
	 * @return
	 */
	private static int heading() {
		double angle = odo.getXYT()[2];
		if (angle < 45 || angle > 315) {
			return 0; // 0
		}
		else if (angle > 45 && angle < 135) {
			return 1; // 90
		}
		else if (angle > 135 && angle < 225) {
			return 2; // 180
		}
		else {
			return 3; // 270
		}
	}
	/**
	 * This method returns the orientation of the tunnel
	 * @param tn_ll_x
	 * @param tn_ll_y
	 * @param tn_ur_x
	 * @param tn_ur_y
	 * @return
	 */
	private static boolean tunnelIsVertical( int tn_ll_y, int tn_ur_y, int starting_location, int islandury, int islandlly){ 
		if(starting_location < 2){
			if(tn_ur_y > islandlly){
				return false;
			}else{
				return true;
			}
		}else{
			if(tn_ll_y < islandury){
				return false;
			}else{
				return true;
			}
		}
	}
	/**
	 * This method returns the length of the tunnel
	 * @param tn_ll_x
	 * @param tn_ll_y
	 * @param tn_ur_x
	 * @param tn_ur_y
	 * @return
	 */
	private static int tunnelLength(int tn_ll_x, int tn_ll_y, int tn_ur_x, int tn_ur_y){ 
		if(tn_ur_y - tn_ll_y == 1 && tn_ur_x - tn_ll_x == 1){
			return 1;
		}else{
			return 2;
		}
	}
	
	/**
	 * This method indicates whether the robot should travel to LL tunnel or UR tunnel from the starting corner.
	 * @param tn_ll_x
	 * @param tn_ll_y
	 * @param tn_ur_x
	 * @param tn_ur_y
	 * @param starting_corner
	 * @param tunnelIsVertical
	 * @return
	 */
	private static boolean toLL(int tn_ll_x, int tn_ll_y, int tn_ur_x, int tn_ur_y, int starting_corner, boolean tunnelIsVertical) {
		if (starting_corner == 0) return true;
		else if (starting_corner == 1) {
			if (tunnelIsVertical) return true;
			else return false;
		}
		else if (starting_corner == 2) {
			return false;
		}
		else {
			if (tunnelIsVertical) return false;
			else return true;
		}
	}
}