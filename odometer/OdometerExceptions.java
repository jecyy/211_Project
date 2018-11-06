package odometer;

/**
 * This class throws error message if more than one odometer instances exist at the same time.
 * @author jecyy
 *
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {

  public OdometerExceptions(String Error) {
    super(Error);
  }

}
