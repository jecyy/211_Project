package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.Odometer;

/**
 * This class implements the traveling algorithm,
 * the robot will only run along the X-axis and Y-axis directions
 * so that the odometer is corrected every time the robot passes a tile.
 * @author jecyy
 *
 */
public class Navigation {

	public static int[] leftaxis = new int[2];
	public static int[] rightaxis = new int[2]; // position of the light sensors
	private static EV3LargeRegulatedMotor leftM;
	private static EV3LargeRegulatedMotor rightM;
	private static double leftR, rightR, trac;
	private static Odometer odo;
	private static double lightTrac = 7.5;
	private static double pi = Math.PI;
	private static int ROTATE_SPEED = 75;

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
			double leftRadius, double rightRadius, double track, Odometer odometer) {
		leftM = leftMotor;
		rightM = rightMotor;
		leftR = leftRadius;
		rightR = rightRadius;
		trac = track;
		odo = odometer;
	}

	/**
	 * This method uses the distance of the two light sensors along the expected direction of traveling
	 * and calculates the angle that the robot should turn in order to correct its orientation
	 * @param b
	 */
	private static void adjustOrientation(double b) {
		double adjust = Math.asin(b / lightTrac) * 180 / pi;
		turn(adjust);
	}
	
	/**
	 * The robot will turn a given angle when this method is called
	 * @param theta
	 */
	private static void turn(double theta) {
		leftM.setSpeed(ROTATE_SPEED);
		rightM.setSpeed(ROTATE_SPEED);
		leftM.rotate(convertAngle(leftR, trac, theta), true);
		rightM.rotate(-convertAngle(rightR, trac, theta), false);
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
}
