import com.ibm.jc.JavaChallenge;
import com.ibm.rally.Car;
import com.ibm.rally.ICar;
import com.ibm.rally.IObject;
import com.ibm.rally.World;

class Vec2d {
	public static final Vec2d ZERO = new Vec2d(0, 0);
	public static final Vec2d NORTH = new Vec2d(0, -1);
	public Vec2d(double x, double y) {
		this.x = x;
		this.y = y;
	}
	public static Vec2d ofPolar(double magnitude, double direction){
		double radDir = direction / 180 * Math.PI;
		return new Vec2d(magnitude * Math.sin(radDir), -magnitude * Math.cos(radDir));
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
	 * Converts screen space coordinates to cartesian
	 * @return
	 */
	public Vec2d toCartesian(){
		return new Vec2d(-y, x);
	}

	public double crossProduct(Vec2d other){
		return x * other.y - other.x * y;
	}

	/**
	 * Gets the bearing of another point in relation to the current point
	 * @param other
	 * @return
	 */
	public double getAngleTo(Vec2d other){
		Vec2d diff = other.subtract(this);
		return diff.getDirection();
	}

	/**
	 * Gets the angle of the current vector. North is 0 degrees
	 * @return
	 */
	private double getUnitDirection(){
		double ra = 180 * Math.acos(x) / Math.PI - 90;
		if(x < 0){
			if(y < 0){
				return -ra;
			}else{
				return -180 + ra;
			}
		}else{
			if(y < 0){
				return -ra;
			}else{
				return 180 + ra;
			}
		}
	}

	/**
	 * Gets the compass bearing of the current vector
	 * @return
	 */
	public double getDirection(){
		return normalize().getUnitDirection();
	}
	public double x, y;

	public static double getAngleDifference(double a1, double a2){
		if(Math.abs(a1 - a2) < Math.abs(a1 - a2 - 360)){
			return a1 - a2;
		}else{
			return a1 - a2 - 360;
		}
	}
}

/**
 * This is the class that you must implement to enable your car within
 * the CodeRally track. Adding code to these methods will give your car
 * it's personality and allow it to compete.
 */
@JavaChallenge(name="Hornet", organization="Bees")
public class RallyCar extends Car {
	/**
	 * @see com.ibm.rally.Car#getColor()
	 */
	public byte getColor() {
		return CAR_YELLOW;
	}

	private int totalCheckpoints;
	private int lastCheckpoint = 0;

	/**
	 * @see com.ibm.rally.Car#initialize()
	 */
	public void initialize() {
		// put implementation here
		totalCheckpoints = World.getCheckpoints().length;
	}


	/**
	 * @see com.ibm.rally.Car#move(int, boolean, ICar, ICar)
	 */
	public void move(int lastMoveTime, boolean hitWall, ICar collidedWithCar, ICar hitBySpareTire) {
		// put implementation here
		setThrottle(100);

		if(getPreviousCheckpoint() == lastCheckpoint){
			lastCheckpoint = (lastCheckpoint + 1) % totalCheckpoints;
		}
		IObject chk = World.getCheckpoints()[lastCheckpoint];
		Vec2d dep = new Vec2d(chk.getX(), chk.getY());
		turnTo(dep);
	}

	public void turnTo(Vec2d pos){
		Vec2d dirVector = Vec2d.ofPolar(1, this.getHeading());
		Vec2d curPos = new Vec2d(getX(), getY());
		double cHeading = dirVector.getDirection();
		double angle = curPos.getAngleTo(pos);
		double diff = Vec2d.getAngleDifference(cHeading, angle);
		if(diff < -10){
			setSteeringSetting(MAX_STEER_RIGHT);
		}else if(diff > 10){
			setSteeringSetting(MAX_STEER_LEFT);
		}else{
			setSteeringSetting(0);
		}
	}
}