package lightSensor;

import lejos.robotics.SampleProvider;

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
