package project;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * This class is used to get data from the Wifi server.
 * @author jecyy
 *
 */
public class Wifi {
	private static final String SERVER_IP = "192.168.2.3";
	private static final int TEAM_NUMBER = 22;
	
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
	
	  @SuppressWarnings("rawtypes")
	  
	  public static Map readData() {
		  WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
		  Map data = null;
		  try {
			data = conn.getData();
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		 
		  
		  return data;
	  }
}
