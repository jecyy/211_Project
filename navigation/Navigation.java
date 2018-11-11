package navigation;

import grasping.Grasp;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lightSensor.LightSensorLeft;
import lightSensor.LightSensorRight;
import odometer.Odometer;
import unload.Unload;

/**
 * This class implements the traveling algorithm,
 * the robot will only run along the X-axis and Y-axis directions
 * so that the odometer is corrected every time the robot passes a tile.
 * @author jecyy
 *
 */
public class Navigation {

	public static int[] leftaxis = new int[2];  // TODO: be consistent with starting position 
	public static int[] rightaxis = new int[2]; // grid position of the light sensors
	private static EV3LargeRegulatedMotor leftM;
	private static EV3LargeRegulatedMotor rightM;
	private static double radius, trac;
	private static Odometer odo;
	private static double lightTrac = 7.5; // TODO: measure the distance between two light sensors
	private static double pi = Math.PI;
	private static final int ROTATE_SPEED = 75;
	private static final int FORWARD_SPEED = 200;
	private static final double ts = 30.48;
	private static final int black = 300; // threshold for black line
	private static final double offset = 10; // TODO: the distance between the center of light-track and the center of rotation

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
			int tn_ll_x, int tn_ll_y, int tn_ur_x, int tn_ur_y, int starting_corner, int Tx, int Ty) {
		leftM = leftMotor;
		rightM = rightMotor;
		radius = Radius;
		trac = track;
		odo = odometer;
		
		final int startx, starty;
		// adjust leftaxis ,rightaxis first
		if (starting_corner == 0) {
			leftaxis[0] = 0;
			leftaxis[1] = 0;
			rightaxis[0] = 0;
			rightaxis[1] = 0;
			startx = 0;
			starty = 0;
		}
		else if (starting_corner == 1) {
			leftaxis[0] = 14;
			leftaxis[1] = 0;
			rightaxis[0] = 14;
			rightaxis[1] = 0;
			startx = 14;
			starty = 0;
		}
		else if (starting_corner == 2) {
			leftaxis[0] = 14;
			leftaxis[1] = 8;
			rightaxis[0] = 14;
			rightaxis[1] = 14;
			startx = 14;
			starty = 8;
		}
		else { // starting_corner == 3
			leftaxis[0] = 0;
			leftaxis[1] = 8;
			rightaxis[0] = 0;
			rightaxis[1] = 8;
			startx = 0;
			starty = 8;
		}
		
		
		// in the very beginning, the robot should be located around the center of the starting corner
		// so we need to travel to the tunnel first
		int tunnelx , tunnely; // the tile we need to get to before entering the tunnel
		if (tn_ur_y - tn_ll_y == 1) { // tunnel is along x-axis
			tunnelx = tn_ll_x - 1;
			tunnely = tn_ll_y;
		}
		else { // tunnel is along y-axis
			tunnelx = tn_ll_x;
			tunnely = tn_ll_y - 1;
		}
		// travel to the tunnel
		travelTo(tunnelx, tunnely);
		// now we have to head to the direction of the tunnel
		if (tn_ll_x - tunnelx == 1) {
			turn(90 - odo.getXYT()[2]);
		}
		else {
			turn(0 - odo.getXYT()[2]);
		}
		// now the robot should be heading to the tunnel
		// since the tunnel is 2 ts long, we make the robot travel for 3 ts
		travelDistance(3 * ts + 2);
		// update leftaxis and rightaxis
		if (tn_ur_y - tn_ll_y == 1) { // tunnel is along x-axis
			leftaxis[0] = tn_ur_x;
			leftaxis[1] = tn_ur_y - 1;
			rightaxis[0] = tn_ur_x;
			rightaxis[1] = tn_ur_y - 1;
		}
		else { // tunnel is along y-axis
			leftaxis[0] = tn_ur_x - 1;
			leftaxis[1] = tn_ur_y;
			rightaxis[0] = tn_ur_x - 1;
			rightaxis[1] = tn_ur_y;
		}
		// now the robot should be positioned at the exit of the tunnel
		// the next step is to travel the robot to the tree
		int treex = Tx - 1, treey = Ty - 1;
		travelTo(treex, treey);
		Grasp.grasp(leftM, rightM, radius, radius, trac, odo);
		// after the grasping, the robot should travel back to the starting position
		// first, let's go back to the tunnel
		if (tn_ur_y - tn_ll_y == 1) { // tunnel is along x-axis
			tunnelx = tn_ur_x;
			tunnely = tn_ll_y - 1;
		}
		else { // tunnel is along y-axis
			tunnelx = tn_ur_x - 1;
			tunnely = tn_ll_y;
		}
		travelTo(tunnelx, tunnely);
		// head to tunnel
		if (tn_ur_y - tunnely == 1) { // tunnel along x-axis
			turn(270 - odo.getXYT()[2]);
		}
		else {
			turn(180 - odo.getXYT()[2]);
		}
		// go thorugh the tunnel
		travelDistance(3 * ts + 2);
		// update leftaxis and rightaxis
		if (tn_ur_y - tn_ll_y == 1) { // tunnel is along x-axis
			leftaxis[0] = tn_ll_x - 1;
			leftaxis[1] = tn_ur_y;
			rightaxis[0] = tn_ur_x - 1;
			rightaxis[1] = tn_ur_y;
		}
		else { // tunnel is along y-axis
			leftaxis[0] = tn_ur_x;
			leftaxis[1] = tn_ur_y - 1;
			rightaxis[0] = tn_ur_x;
			rightaxis[1] = tn_ur_y - 1;
		}
		// now the robot should be positioned at the entrance of the tunnel with leftaxis and rightaxis updated
		// next go back to the starting corner
		travelTo(startx, starty);
		
		Unload.unload();
	}

	/**
	 * This method travels the robot to the center of the tile located at (x, y),
	 * where the point (x, y) should be the lower left vertex of the tile.
	 * @param x
	 * @param y
	 */
	private static void travelTo(int x, int y) {
		int currentx = leftaxis[0], currenty = leftaxis[1];
		int xdis = x - currentx, ydis = y - currenty;
		// x direction
		if (xdis >= 0) {
			turn(90 - odo.getXYT()[2]); // turn to 90 degree
		}
		else {
			turn(270 - odo.getXYT()[2]); // turn to 270 degree
		}
		lineTravel(Math.abs(xdis));
		// y direction
		if (ydis > 0) {
			turn(0 - odo.getXYT()[2]); // turn to 0 degree
		}
		else {
			turn(180 - odo.getXYT()[2]); // turn to 180 degree
		}
		lineTravel(Math.abs(ydis));
	}
	
	/**
	 * This method travels the robot for a given distance forward, without ant correction.
	 * @param s
	 */
	private static void travelDistance(double s) {
		leftM.setSpeed(FORWARD_SPEED);
		rightM.setSpeed(FORWARD_SPEED);
		leftM.rotate(convertDistance(radius, s));
		rightM.rotate(convertDistance(radius, s));
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
	    long left_time = 0, right_time = 0;
		go();
		boolean left_passed = false, right_passed = false; // indicates that the left/right light sensor has passed a line
		while(!(left_passed && right_passed)) { // if loop breaks when both sensors have seen the line
			if (!left_passed && LightSensorLeft.get_light() < black) {
				left_passed = true;
			    left_time = System.currentTimeMillis();
			}
			if (!right_passed && LightSensorRight.get_light() < black) {
				right_passed = true;
			    right_time = System.currentTimeMillis();
			}
		}
		freeze();
		long dif = left_time - right_time; // time difference between the two detections
		double d = FORWARD_SPEED * 1000 * dif * pi / 180 * radius; // distance traveled by the robot within dif
		adjustOrientation(d); // adjust the orientation
		freeze();
		// correct the odometer orientation &
		// correct the odometer position
		// since we've recorded the estimated position of the two light sensors,
		// we can use them to calculate the position of center of rotation using an offset
		if (heading() == 0) {
			odo.setTheta(0);
			odo.setY((leftaxis[1] + rightaxis[1])/2 + offset);
		}
		else if (heading() == 1) {
			odo.setTheta(90);
			odo.setX((leftaxis[0] + rightaxis[0])/2 + offset);
		}
		else if(heading() == 2) {
			odo.setTheta(180);
			odo.setY((leftaxis[1] + rightaxis[1])/2 - offset);
		}
		else {
			odo.setTheta(270);
			odo.setX((leftaxis[0] + rightaxis[0])/2 - offset);
		}
	}

	/**
	 * This method uses the distance of the two light sensors along the expected direction of traveling
	 * and calculates the angle that the robot should turn in order to correct its orientation
	 * @param b
	 */
	private static void adjustOrientation(double d) {
		double adjust = Math.asin(d / lightTrac) * 180 / pi;
		turn(adjust);
	}
	
	/**
	 * The robot will turn a given angle when this method is called
	 * @param theta
	 */
	private static void turn(double theta) {
		leftM.setSpeed(ROTATE_SPEED);
		rightM.setSpeed(ROTATE_SPEED);
		leftM.rotate(convertAngle(radius, trac, theta), true);
		rightM.rotate(-convertAngle(radius, trac, theta), false);
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
		leftM.forward();
		rightM.forward();
	}
	
	/**
	 * This method returns the expected heading of the robot currently.
	 * (0 for 0 degree, 1 for 90 degree, 2 for 180 degree, 3 for 270 degree)
	 * @return
	 */
	private static int heading() {
		double angle = odo.getXYT()[2];
		if (angle < 45 && angle > 315) {
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
}