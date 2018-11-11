package lightSensor;

import lejos.robotics.SampleProvider;
import navigation.Navigation;

/**
 * This class provides the left light sensor readings.
 * @author jecyy
 *
 */

// TODO: change this class according to LightSensorRight
public class LightSensorLeft extends Thread {
	private static float light = 0;
	private SampleProvider myLightSample;
	private float[] sampleLight;
	private static final int black = 300;
	private static double angle;
	private static final double ts = 30.48;
	
	/**
	 * Default constructor
	 * @param myLightSample
	 * @param sampleLight
	 */
	public LightSensorLeft(SampleProvider myLightSample, float[] sampleLight) {
		this.myLightSample = myLightSample;
		this.sampleLight = sampleLight;
	}
	
	/**
	 * This method converts the light sensor reading into a float ranging [0, 1000]
	 */
	public void run() {
		while (true) {
			myLightSample.fetchSample(sampleLight, 0); // activate the light sensor
			light = sampleLight[0] * 1000; // get the light sensor reading
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
			
			// the position of the left-side light sensor is updated here
			if (light < black) { // black line detected
				if (angle < 45 && angle > 315) {
					Navigation.leftaxis[1] += ts;
				}
				else if (angle > 45 && angle < 135) {
					Navigation.leftaxis[0] += ts;
				}
				else if (angle > 135 && angle < 225) {
					Navigation.leftaxis[1] -= ts;
				}
				else {
					Navigation.leftaxis[0] -= ts;
				}
			}
		}
	}

	/**
	 * This method is to read the current sensor reading
	 * @return
	 */
	public static float get_light() {
		return light;
	}
}
