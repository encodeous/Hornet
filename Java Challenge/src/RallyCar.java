import com.ibm.jc.JavaChallenge;
import com.ibm.rally.Car;
import com.ibm.rally.ICar;
import com.ibm.rally.IObject;
import com.ibm.rally.World;

import java.util.*;

class Vec2d implements Comparable<Vec2d>{
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

	public int compareTo(Vec2d o) {
		if(x == o.x){
			return Double.compare(y, o.y);
		}
		return Double.compare(x, o.x);
	}
}

class Pair<T, V>{
	public T x;
	public V y;

	public Pair(T x, V y) {
		this.x = x;
		this.y = y;
	}
}

class CarPath{
	public ArrayDeque<Vec2d> points = new ArrayDeque<>();

	public static ArrayDeque<Vec2d> chaikinIter(double subdivideAmount, ArrayDeque<Vec2d> input){
		ArrayDeque<Vec2d> output = new ArrayDeque<>();
		Iterator<Vec2d> itr = input.iterator();
		Vec2d prev = null;
		output.add(input.getFirst());
		while(itr.hasNext()){
			if(prev == null){
				prev = itr.next();
			}
			else{
				Vec2d cur = itr.next();
				double dist = cur.distanceTo(prev);
				Vec2d dir = cur.subtract(prev).normalize();
				output.add(prev.add(dir.multiply(dist * subdivideAmount)));
				output.add(cur.add(dir.multiply(dist * -subdivideAmount)));
				prev = cur;
			}
		}
		output.add(input.getLast());
		return output;
	}
}

class PointComparator implements Comparator<Pair<Integer, Vec2d>>{

	@Override
	public int compare(Pair<Integer, Vec2d> o1, Pair<Integer, Vec2d> o2) {
		if(o1.x.equals(o2.x)){
			return o1.y.compareTo(o2.y);
		}
		return Integer.compare(o1.x, o2.x);
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
	public final static int MAX_X = 1010;
	public final static int MAX_Y = 580;
	public final static int BLOCK_SZ = 10;
	private int getBlock(int dim){
		return dim / BLOCK_SZ;
	}
	private Vec2d getLocation(IObject obj){
		return new Vec2d(obj.getX(), obj.getY());
	}
	private Vec2d getBlockLocation(Vec2d mapLocation){
		return new Vec2d(getBlock((int)mapLocation.x), getBlock((int)mapLocation.y));
	}
	private double[][] grid = new double[getBlock(MAX_X) + 1][getBlock(MAX_Y) + 1];
	private double[][] dist = new double[getBlock(MAX_X) + 1][getBlock(MAX_Y) + 1];
	private boolean[][] visited = new boolean[getBlock(MAX_X) + 1][getBlock(MAX_Y) + 1];
	private Vec2d nextCheckpoint;

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
//		setSteeringSetting(MAX_STEER_RIGHT);

		if(getPreviousCheckpoint() == lastCheckpoint){
			lastCheckpoint = (lastCheckpoint + 1) % totalCheckpoints;
		}
		IObject chk = World.getCheckpoints()[lastCheckpoint];
		Vec2d dep = new Vec2d(chk.getX(), chk.getY());
		turnTo(dep);
		System.out.println(getSpeed());
	}

	public double get(Vec2d block){
		if(block.x < 0 || block.x >= MAX_X || block.y < 0 || block.y >= MAX_Y)
			return Integer.MAX_VALUE;
		return grid[(int)block.x][(int)block.y];
	}

	public void set(Vec2d block, double value){
		if(block.x < 0 || block.x >= MAX_X || block.y < 0 || block.y >= MAX_Y) return;
		grid[(int)block.x][(int)block.y] = value;
	}

	public void updateMap(){
		for(double[] arr : grid){
			Arrays.fill(arr, 0);
		}
		for(double[] arr : dist){
			Arrays.fill(arr, 0);
		}
		for(boolean[] arr : visited){
			Arrays.fill(arr, false);
		}
		for(ICar car : getOpponents()){
			Vec2d pos = getLocation(car);
			Vec2d block = getBlockLocation(pos);
			for(int x = -3; x <= 3; x++){
				for(int y = -3; y <= 3; y++){
					Vec2d dVec = new Vec2d(x, y);
					Vec2d nVec = dVec.add(block);
					double distance = nVec.distanceTo(block);
					if(distance < 3){
						double weight = (3 - distance) / 3;
						set(nVec, get(nVec) + weight);
					}
				}
			}
		}
	}

	public Vec2d getCurrentLocation(){
		return new Vec2d(getX(), getY());
	}

	public Vec2d getCurrentDirection(){
		return Vec2d.ofPolar(1, this.getHeading());
	}

	public Vec2d getPredictedLocation(){
		return getCurrentLocation().add(getCurrentDirection().multiply(getSpeed()));
	}

	public CarPath getPathTo(Vec2d dest){
		PriorityQueue<Pair<Integer, Vec2d>> pq = new PriorityQueue<>(new PointComparator());
		CarPath path = new CarPath();
		path.points.addLast(getCurrentLocation());
		Vec2d start = getPredictedLocation();
		pq.add(new Pair<>(0, start));
		while(pq.size() != 0){
			
		}
		return path;
	}

	public void turnTo(Vec2d pos){
		Vec2d dirVector = getCurrentDirection();
		Vec2d curPos = getCurrentLocation();
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