package usSensor;

/**
 * This interface controls the class named UltrasonicPoller
 * and contains two abstract methods which are processUSData(int distance) and readUSDistance(). 
 * @author jecyy
 *
 */
public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
