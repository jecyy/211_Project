package project;

import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to show the color sensor reading for the color classification mode
 */
public class Display implements Runnable{
	private TextLCD lcd;

	/**
	 * This is the class constructor
	 */
	public Display(TextLCD lcd) {
		this.lcd = lcd;
	}

	public void run() {    
		lcd.clear();
		while (true) {
			lcd.clear();
 //TODO: determine the content
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
			}
		}
	}
}

