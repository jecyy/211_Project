
package usSensor;

import lejos.robotics.SampleProvider;

/**
 * This class runs the ultrasonic sensor using a thread
 * and provides the readings.
 * @author jecyy
 *
 */
public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private float[] usData;
	private static int distance;

	/**
	 * Class constructor
	 * @param us
	 * @param usData
	 */
	public UltrasonicPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
	}

	/**
	 * This method transfers the ultrasonic sensor readings into integers ranging [2, 200]
	 */
	public void run() {
		while (true) {
			us.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			if (distance > 200) distance = 200; // if distance is too large, it is set to 200
			if (distance < 2) distance = 2; // if distance is too small, it is set to 4
			try {
				Thread.sleep(20);
			} catch (Exception e) {
			}
		}
	}

	/**
	 * This method is to read the current sensor reading
	 * @return
	 */
	public static int get_distance() {
		return distance;
	}
}