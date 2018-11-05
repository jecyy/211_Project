package odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class runs the odometer, recording the current position of the robot 
 * @author jecyy
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
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
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
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
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
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
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
      odo.update(dX,dY,dT);                                                           // Update odometer values with new calculated values

      // this ensures that the odometer only runs once every period
      try {
        Thread.sleep(ODOMETER_PERIOD);
      } catch (InterruptedException e) {
        // there is nothing to be done
      }
    }
  }

}
