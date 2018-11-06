package project;

import lejos.hardware.lcd.TextLCD;

/**
 * This class runs the display as a thread 
 * and specifies what need to be on the screen.
 * @author jecyy
 *
 */
public class Display implements Runnable{
	private TextLCD lcd;

	/**
	 * Class constructor
	 * @param lcd
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

