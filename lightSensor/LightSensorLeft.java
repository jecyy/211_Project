package lightSensor;

import lejos.robotics.SampleProvider;
import odometer.Odometer;

/**
 * This class provides the left light sensor readings
 * @author jecyy
 *
 */

// TODO: change this class according to LightSensorRight
public class LightSensorLeft extends Thread {
	private static float light = 0;
	private SampleProvider myLightSample;
	private float[] sampleLight;
	private Odometer odo;
	
	public LightSensorLeft(SampleProvider myLightSample, float[] sampleLight) {
		this.myLightSample = myLightSample;
		this.sampleLight = sampleLight;
	}
	
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
