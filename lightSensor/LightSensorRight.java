package lightSensor;

import lejos.robotics.SampleProvider;
import navigation.Navigation;
import odometer.Odometer;

/**
 * This class provides the right light sensor readings
 * @author jecyy
 *
 */
public class LightSensorRight extends Thread {
	private static float light = 0;
	private SampleProvider myLightSample;
	private float[] sampleLight;
	private Odometer odo;
	private static int black = 300; // black threshold
	private static double angle;
	private static double ts = 30.48;
	
	public LightSensorRight(SampleProvider myLightSample, float[] sampleLight, Odometer odometer) {
		this.myLightSample = myLightSample;
		this.sampleLight = sampleLight;
		this.odo = odometer;
	}
	
	public void run() {
		while (true) {
			myLightSample.fetchSample(sampleLight, 0); // activate the light sensor
			light = sampleLight[0] * 1000; // get the light sensor reading
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
			
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
