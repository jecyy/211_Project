package odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class runs the odometer, recording the current position and orientation of the robot 
 * @authors jecyy, PaulHooley
 *
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount; // Tacho L at current sample
  private int rightMotorTachoCount; // Tacho R at current sample
  private int leftMotorLastTacho; // Tacho L at last sample
  private int rightMotorLastTacho; // Tacho R at last sample
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;
  private final double pi = Math.PI;//



  private static final long ODOMETER_PERIOD = 20; // odometer update period in ms

  /**
   * This is the default constructor which initiates all motors and relevant variables
   * @param leftMotor
   * @param rightMotor
   * @param TRACK
   * @param WHEEL_RAD
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is to ensure that only one odometer is used throughout the program
   * @param leftMotor
   * @param rightMotor
   * @param TRACK
   * @param WHEEL_RAD
   * @return
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This method returns the current odometer instance (throws error if there is none)
   * @return
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method updates the odometer based on the tacho count of wheels
   */
  public void run() {
    double distL,distR,deltaD,dX,dY,dT;//
    double T;
    T = odo.getXYT()[2];
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    leftMotorLastTacho = leftMotor.getTachoCount();
    rightMotorLastTacho = rightMotor.getTachoCount();

    while (true) {
      leftMotorTachoCount = leftMotor.getTachoCount(); // get tacho counts
      rightMotorTachoCount = rightMotor.getTachoCount();

      // Calculate new robot position based on tachometer counts
      distL = pi * this.WHEEL_RAD * (leftMotorTachoCount - leftMotorLastTacho) / 180; // computer wheel displacements
      distR = pi * this.WHEEL_RAD * (rightMotorTachoCount - rightMotorLastTacho) / 180;
      leftMotorLastTacho = leftMotorTachoCount;                                       // save tacho counts for next iteration
      rightMotorLastTacho = rightMotorTachoCount;
      deltaD = 0.5 * (distL + distR);                                                 // compute vehicle displacement
      dT = ((distL - distR) / this.TRACK) * 360 / (2 * pi);                           // compute change in heading
      T += dT;                                                                        // update heading
      dX = deltaD * Math.sin(T * 2 * pi / 360);                                       // compute X component of displacement
      dY = deltaD * Math.cos(T * 2 * pi / 360);                                       // compute Y component of displacement
      odo.update(-dX,-dY,-dT);                                                           // Update odometer values with new calculated values

      // this ensures that the odometer only runs once every period
      try {
        Thread.sleep(ODOMETER_PERIOD);
      } catch (InterruptedException e) {
        // there is nothing to be done
      }
    }
  }

}
