package lightSensor;

import lejos.robotics.SampleProvider;

/**
 * This class provides the right light sensor readings
 * @author jecyy
 *
 */
public class LightSensorRight extends Thread {
	private static float light = 0;
	private SampleProvider myLightSample;
	private float[] sampleLight;
	
	/**
	 * Default constructor
	 * @param myLightSample
	 * @param sampleLight
	 * @param odometer
	 */
	public LightSensorRight(SampleProvider myLightSample, float[] sampleLight) {
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
