import com.ibm.jc.JavaChallenge;
import com.ibm.rally.Car;
import com.ibm.rally.ICar;

class Vec2d {
	public static final Vec2d ZERO = new Vec2d(0, 0);
	public static final Vec2d NORTH = new Vec2d(0, -1);
	public Vec2d(double x, double y) {
		this.x = x;
		this.y = y;
	}
	public static Vec2d ofPolar(double magnitude, double direction){
		double radDir = direction / 180 * Math.PI;
		return new Vec2d(magnitude * Math.cos(radDir), magnitude * Math.sin(radDir));
	}
	public Vec2d(Vec2d other){
		this.x = other.x;
		this.y = other.y;
	}

	/**
	 * Adds another vector to the current vector.
	 * @param other
	 * @return
	 */
	public Vec2d add(Vec2d other){
		return new Vec2d(x + other.x, y + other.y);
	}

	/**
	 * Subtracts another vector from the current vector
	 * @param other
	 * @return
	 */
	public Vec2d subtract(Vec2d other){
		return new Vec2d(x - other.x, y - other.y);
	}

	/**
	 * Multiplies the vector by a scalar
	 * @param scalar
	 * @return
	 */
	public Vec2d multiply(double scalar){
		return new Vec2d(x * scalar, y * scalar);
	}

	/**
	 * Calculates the distance to another point.
	 * @param point
	 * @return
	 */
	public double distanceTo(Vec2d point){
		double dx = x - point.x, dy = y - point.y;
		return Math.sqrt(dx * dx + dy * dy);
	}

	/**
	 * Calculates the magnitude of the vector
	 * @return
	 */
	public double magnitude(){
		return Math.sqrt(x * x + y * y);
	}

	/**
	 * Gets a normalized vector of the current one
	 * @return
	 */
	public Vec2d normalize(){
		double mag = magnitude();
		return new Vec2d(x / mag, y / mag);
	}

	/**
	 * Calculates the dot product with another vector
	 * @param other
	 * @return
	 */
	public double dotProduct(Vec2d other){
		return x * other.x + y * other.y;
	}

	/**
	 * Calculates the angle to a given point (assuming they are both originating from the origin)
	 * @param otherPoint
	 * @return the angle in degrees
	 */
	public double getAngleTo(Vec2d otherPoint){
		return 180 * Math.acos(this.dotProduct(otherPoint) / (magnitude() * otherPoint.magnitude())) / Math.PI;
	}

	/**
	 * Gets the compass bearing of the current vector
	 * @return
	 */
	public double getDirection(){
		return getAngleTo(NORTH);
	}

	public double x, y;
}

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
		return CAR_YELLOW;
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
		setSteeringSetting(MAX_STEER_LEFT);
		setThrottle(MAX_THROTTLE);
		Vec2d cur = Vec2d.ofPolar(this.getSpeed(), this.getHeading());
		System.out.println(cur.getDirection());
	}
}