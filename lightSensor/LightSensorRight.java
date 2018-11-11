package lightSensor;

import lejos.robotics.SampleProvider;
import navigation.Navigation;

/**
 * This class provides the right light sensor readings
 * @author jecyy
 *
 */
public class LightSensorRight extends Thread {
	private static float light = 0;
	private SampleProvider myLightSample;
	private float[] sampleLight;
	private static final int black = 300; // black threshold
	private static double angle;
	private static final double ts = 30.48;
	
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
			
			// the position of the right-side light sensor is updated here
			if (light < black) { // black line detected
				if (angle < 45 && angle > 315) {
					Navigation.rightaxis[1] += ts;
				}
				else if (angle > 45 && angle < 135) {
					Navigation.rightaxis[0] += ts;
				}
				else if (angle > 135 && angle < 225) {
					Navigation.rightaxis[1] -= ts;
				}
				else {
					Navigation.rightaxis[0] -= ts;
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
