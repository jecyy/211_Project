package navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.Odometer;

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

	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double track, Odometer odometer) {
		leftM = leftMotor;
		rightM = rightMotor;
		leftR = leftRadius;
		rightR = rightRadius;
		trac = track;
		odo = odometer;
	}

	private static void adjustOrientation(double b) {
		double adjust = Math.asin(b / lightTrac) * 180 / pi;
		turn(adjust);
	}
	
	private static void turn(double theta) {
		leftM.setSpeed(ROTATE_SPEED);
		rightM.setSpeed(ROTATE_SPEED);
		leftM.rotate(convertAngle(leftR, trac, theta), true);
		rightM.rotate(-convertAngle(rightR, trac, theta), false);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (pi * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, pi * width * angle / 360.0);
	}
}
