package project;

import java.util.Map;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lightSensor.ColorSensorPoller;
import lightSensor.LightSensorLeft;
import lightSensor.LightSensorRight;
import localization.Localizer;
//import localization.Localizer2;
import navigation.Navigation;
import odometer.Odometer;
import odometer.OdometerExceptions;
import usSensor.UltrasonicPoller;

/**
 * This will be the main class that calls all threads (besides the main thread) 
 * and controls the work flow of the whole process.
 * @authors jecyy, PaulHooley
 *
 */
public class Project {
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port leftLight = LocalEV3.get().getPort("S2");
	private static final Port rightLight = LocalEV3.get().getPort("S3");
	private static final Port portColor = LocalEV3.get().getPort("S4");
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 15.0;


	public static void main(String[] args) throws OdometerExceptions{
		// Odometer related objects
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		// Ultrasonic sensor
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		// leftLight sensor
		@SuppressWarnings("resource")
		SensorModes leftlight = new EV3ColorSensor(leftLight);
		SampleProvider leftLightSample = leftlight.getMode("Red");
		float[] sampleLightleft = new float[leftlight.sampleSize()];
		// rightLight sensor
		@SuppressWarnings("resource")
		SensorModes rightlight = new EV3ColorSensor(rightLight);
		SampleProvider rightLightSample = rightlight.getMode("Red");
		float[] sampleLightright = new float[rightlight.sampleSize()];
		// Color sensor
		@SuppressWarnings("resource")
		SensorModes myColor = new EV3ColorSensor(portColor);
		SampleProvider myColorSample = myColor.getMode("RGB");
		float[] sampleColor = new float[3];
		// returned
		UltrasonicPoller ultrasonic = new UltrasonicPoller(usDistance, usData);
		//UltrasonicPoller ringUltrasonic = new UltrasonicPoller(ringUsDistance, ringUsData);
		LightSensorLeft leftlightsensor = new LightSensorLeft(leftLightSample, sampleLightleft);
		LightSensorRight rightlightsensor = new LightSensorRight(rightLightSample, sampleLightright);
		ColorSensorPoller color = new ColorSensorPoller(myColorSample, sampleColor);

		boolean isRed = false;

		@SuppressWarnings("rawtypes")
		Map data = Wifi.readData();

		int redTeam = ((Long) data.get("RedTeam")).intValue();
		int greenTeam = ((Long) data.get("GreenTeam")).intValue();

		if (redTeam == 22)	//Check if team 13 is red! if not we are green
			isRed = true;
		else
			isRed = false;

		final int starting_corner, llx, lly, urx, ury, islandllx, islandlly, islandurx, islandury, tnllx, tnlly, tnurx, tnury, tx, ty;

		if(isRed) {
			starting_corner = ((Long) data.get("RedCorner")).intValue();
			llx = ((Long) data.get("Red_LL_x")).intValue();
			lly = ((Long) data.get("Red_LL_y")).intValue();
			urx = ((Long) data.get("Red_UR_x")).intValue();
			ury = ((Long) data.get("Red_UR_y")).intValue();
			tnllx = ((Long) data.get("TNR_LL_x")).intValue();
			tnlly = ((Long) data.get("TNR_LL_y")).intValue();
			tnurx = ((Long) data.get("TNR_UR_x")).intValue();
			tnury = ((Long) data.get("TNR_UR_y")).intValue();
			tx = ((Long) data.get("TR_x")).intValue();
			ty = ((Long) data.get("TR_y")).intValue();
		}

		else {
			starting_corner = ((Long) data.get("GreenCorner")).intValue();
			llx = ((Long) data.get("Green_LL_x")).intValue();
			lly = ((Long) data.get("Green_LL_y")).intValue();
			urx = ((Long) data.get("Green_UR_x")).intValue();
			ury = ((Long) data.get("Green_UR_y")).intValue();
			tnllx = ((Long) data.get("TNG_LL_x")).intValue();
			tnlly = ((Long) data.get("TNG_LL_y")).intValue();
			tnurx = ((Long) data.get("TNG_UR_x")).intValue();
			tnury = ((Long) data.get("TNG_UR_y")).intValue();
			tx = ((Long) data.get("TG_x")).intValue();
			ty = ((Long) data.get("TG_y")).intValue();
		}

		islandllx = ((Long) data.get("Island_LL_x")).intValue();
		islandlly = ((Long) data.get("Island_LL_y")).intValue();
		islandurx = ((Long) data.get("Island_UR_x")).intValue();
		islandury = ((Long) data.get("Island_UR_y")).intValue();

//		starting_corner = 0;
//		llx = 0;
//		lly = 0;
//		urx = 4;
//		ury = 4;
//		tnllx = 3;
//		tnlly = 2;
//		tnurx = 5;
//		tnury = 3;
//		tx = 7;
//		ty = 4;
//
//
//		islandllx = 5;
//		islandlly = 0;
//		islandurx = 8;
//		islandury = 8;

			lcd.clear();

			// Start odometer
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			// Start UltrasonicPoller
			Thread UltrasonicThread = new Thread(ultrasonic);
			UltrasonicThread.start();

			(new Thread() {
				public void run() {
					Localizer.run(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, odometer, starting_corner);
				}
			}).start();

			while (Localizer.finished == false)
				; // check if Light Localizer is finished
			Thread colorThread = new Thread(color);
			colorThread.start();
			Thread leftThread = new Thread(leftlightsensor);
			leftThread.start();
			Thread rightThread = new Thread(rightlightsensor);
			rightThread.start();

			(new Thread() { // spawn a new Thread to avoid Search.run() from blocking
				public void run() {
					Navigation.drive(leftMotor, rightMotor, WHEEL_RAD, TRACK, odometer, tnllx, tnlly, tnurx, tnury, starting_corner, tx, ty, islandury, islandlly);
				}
			}).start();

			while (Button.waitForAnyPress() != Button.ID_ESCAPE)
				;
			System.exit(0);
	}

}
