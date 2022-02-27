import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

/*	Main function that calls the filter on the Java side. On the actual rocket, 
	this will be integrated somewhere in the overall code, but it will call my
	USQUE class methods in the same way. Here, main is for dev, testing, and 
	demo purposes.
*/
class Main {
	public static void main(String[] args) {
		String params = new String();
		
		/*	Read in the imu_params.txt file, that describes the noise and bias 
			properties of the gyro and mag, and the initial config of the rocket
			and filter. This should only be done when the rocket is starting up.
			imu_params.txt should be formatted in json
		*/
		try {
			File paramFile = new File("imu_params.txt");
			Scanner myScanner = new Scanner(paramFile);
			while(myScanner.hasNextLine()) {
				params += myScanner.nextLine();
			}
			myScanner.close();
		}
		catch (FileNotFoundException e) {
			System.err.println("imu_params.txt not found");
			e.printStackTrace();
		}
		/* in subsequent steps, params should be updated with the latest 
		 measurements, as well as results of the last filter step (filterStep
		 should return results) */
		params = USQUE.filterStep(params);
		return;
	}
}
