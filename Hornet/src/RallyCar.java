import com.ibm.jc.JavaChallenge;
import com.ibm.rally.Car;
import com.ibm.rally.ICar;
/**
 * This is the class that you must implement to enable your car within
 * the CodeRally track. Adding code to these methods will give your car
 * it's personality and allow it to compete.
 */
@JavaChallenge(name="Hornet",organization="Bees")
public class RallyCar extends Car {
	/**
	 * @see com.ibm.rally.Car#getColor()
	 */
	public byte getColor() {
		return CAR_BLUE;
	}

	/**
	 * @see com.ibm.rally.Car#initialize()
	 */
	public void initialize() {
		// put implementation here
	}

	/**
	 * @see com.ibm.rally.Car#move(int, boolean, ICar, ICar)
	 */
	public void move(int lastMoveTime, boolean hitWall, ICar collidedWithCar, ICar hitBySpareTire) {
		// put implementation here
	}
}