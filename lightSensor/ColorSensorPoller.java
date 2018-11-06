package lightSensor;

import lejos.robotics.SampleProvider;

/**
 * This class runs the color sensor and provides the readings.
 * @author jecyy
 *
 */
public class ColorSensorPoller extends Thread {
	private static float R, G, B;
	private SampleProvider myColorSample;
	private float[] sampleColor;
	
	/**
	 * Default constructor
	 * @param myColorSample
	 * @param sampleColor
	 */
	public ColorSensorPoller(SampleProvider myColorSample, float[] sampleColor) {
		this.myColorSample = myColorSample;
		this.sampleColor = sampleColor;
	}
	
	/**
	 * This method gets the color sensor readings and converts them into floats ranging [0, 255]
	 */
	public void run() {
		while (true) {
			myColorSample.fetchSample(sampleColor, 0); // activate the color sensor
			R = sampleColor[0] * 1000; // get the color sensor reading
			G = sampleColor[1] * 1000;
			B = sampleColor[2] * 1000;
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			}
		}
	}

	/**
	 * This method is to read the current Red value
	 * @return
	 */
	public static float getR() {
		return R;
	}
	
	/**
	 * This method is to read the current Green value
	 * @return
	 */
	public static float getG() {
		return G;
	}
	
	/**
	 * This method is to read the current Blue value
	 * @return
	 */
	public static float getB() {
		return B;
	}
}
