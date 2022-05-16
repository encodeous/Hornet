import com.ibm.jc.JavaChallenge;
import com.ibm.rally.Car;
import com.ibm.rally.ICar;
import com.ibm.rally.IObject;
import com.ibm.rally.World;

import java.util.*;

/**
 * @author Adam Chen
 * Date: 5/16/2022
 * My submission to ICS3/CodeRally. Main code starts at line 788.
 * Sadly, I could only get a single file to run in CodeRally, so gl!
 */

/**
 * 2 dimensional vector extensions, used for calculating EVERYTHING :)
 */
class Vec2d implements Comparable<Vec2d>{
	public Vec2d(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Converts a polar vector into a cartesian one
	 * @param magnitude the magnitude
	 * @param direction the direction
	 * @return a cartesian vector
	 */
	public static Vec2d ofPolar(double magnitude, double direction){
		double radDir = direction / 180 * Math.PI;
		return new Vec2d(magnitude * Math.sin(radDir), -magnitude * Math.cos(radDir));
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

	/**
	 * Gets the smaller angle difference between two headings
	 * @param a1 heading 1
	 * @param a2 heading 2
	 * @return the difference added to a1 to achieve a2
	 */
	public static double getAngleDifference(double a1, double a2){
		if(a1 < 0 != a2 < 0){
			double c1 = Math.abs(a1) + Math.abs(a2);
			double c2 = 360 - Math.abs(a1) - Math.abs(a2);
			if(c1 < c2){
				if(a1 < a2){
					return c1;
				}else{
					return -c1;
				}
			}else{
				if(a1 > a2){
					return c2;
				}else{
					return -c2;
				}
			}
		}else{
			if(Math.abs(a1 - a2) < Math.abs(a2 - a1)){
				return a1 - a2;
			}else{
				return a2 - a1;
			}
		}
	}

	public int compareTo(Vec2d o) {
		if(x == o.x){
			return Double.compare(y, o.y);
		}
		return Double.compare(x, o.x);
	}

	@Override
	public String toString() {
		return "Vec2d{" +
				"x=" + x +
				", y=" + y +
				'}';
	}

	/**
	 * Gets the distance between a line and a point
	 * @param ep1 p1 of the line
	 * @param ep2 p2 of the line
	 * @param pt the point
	 * @return the distance
	 */

	public static double getLineDist(Vec2d ep1, Vec2d ep2, Vec2d pt){
		// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		double dx1 = ep2.x - ep1.x;
		double dy1 = ep2.y - ep1.y;
		return Math.abs(dx1 * (ep1.y - pt.y) - (ep1.x - pt.x) * (dy1)) / Math.sqrt(dx1 * dx1 + dy1 * dy1);
	}
}

/**
 * Just a regular ol' pair/tuple class that java doesn't like providing
 * @param <T> The first type
 * @param <V> The second type
 */
class Pair<T, V>{
	public T x;
	public V y;

	public Pair(T x, V y) {
		this.x = x;
		this.y = y;
	}
}

/**
 * An object that represents a predicted path of the car
 */
class CarPath{
	/**
	 * The 2d world points of the path
	 */
	public ArrayDeque<Vec2d> points = new ArrayDeque<>();
	/**
	 * The length of the path
	 */
	public double length = 0;
	/**
	 * The weight of the path
	 */
	public double weight = 0;
	/**
	 * The ending position in the path
	 */
	public Vec2d destination;

	@Override
	public String toString() {
		return "CarPath{" +
				"points=" + points +
				", length=" + length +
				", weight=" + weight +
				", destination=" + destination +
				'}';
	}
}

/**
 * A class used to compare nodes during the pathfinding
 */
class PointComparator implements Comparator<Pair<Double, Vec2d>>{

	@Override
	public int compare(Pair<Double, Vec2d> o1, Pair<Double, Vec2d> o2) {
		if(o1.x.equals(o2.x)){
			return o1.y.compareTo(o2.y);
		}
		return Double.compare(o1.x, o2.x);
	}
}

/**
 * A Utility class containing the code for line simplification
 */
class LineUtil {
	/**
	 * Executes the Ramer–Douglas–Peucker algorithm to simplify the number of poly-lines in the path of the vehicle
	 * @param epsilon the degree of "simpleness" of the line
	 * @param input a list of points representing a poly-line
	 * @return
	 */

	public static ArrayDeque<Vec2d> simplify(double epsilon, ArrayDeque<Vec2d> input){
		if(input.size() <= 1) return input;
		Vec2d p1 = input.getFirst();
		Vec2d p2 = input.getLast();
		ArrayDeque<Vec2d> arr1 = new ArrayDeque<>();
		ArrayDeque<Vec2d> arr2 = new ArrayDeque<>();
		int idx = 0;
		double md = 0;
		int mdi = 0;
		for(Vec2d point : input){
			if(idx != 0 && idx != input.size() - 1){
				double d = Vec2d.getLineDist(p1, p2, point);
				if(d > md){
					md = d;
					mdi = idx;
				}
			}
			idx++;
		}
		if(md < epsilon || input.size() == 2){
			ArrayDeque<Vec2d> deq = new ArrayDeque<>();
			deq.add(p1);
			deq.add(p2);
			return deq;
		}
		idx = 0;
		for(Vec2d point : input){
			if(idx < mdi){
				arr1.add(point);
			}
			else if(idx == mdi){
				arr1.add(point);
				arr2.add(point);
			}
			else{
				arr2.add(point);
			}
			idx++;
		}
		ArrayDeque<Vec2d> ans = new ArrayDeque<>();
		ArrayDeque<Vec2d> a1 = simplify(epsilon, arr1);
		a1.removeLast();
		ArrayDeque<Vec2d> a2 = simplify(epsilon, arr2);
		for(Vec2d k : a1){
			ans.add(k);
		}
		for(Vec2d k : a2){
			ans.add(k);
		}
		return ans;
	}
}

/**
 * This is the class that you must implement to enable your car within
 * the CodeRally track. Adding code to these methods will give your car
 * it's personality and allow it to compete.
 */
@JavaChallenge(name="The Hornet", organization="Adam Chen")
public class RallyCar extends Car {
	/**
	 * @see com.ibm.rally.Car#getColor()
	 */
	public byte getColor() {
		return CAR_YELLOW;
	}
	// Defines some basic game information

	private int totalCheckpoints;
	private final boolean DEBUG = false;
	// Defines the map dimensions
	public final static int MAX_X = 1010;
	public final static int MAX_Y = 580;
	public final static int BLOCK_SZ = 10;
	// Defines parameters involving collision detection
	public final static int TRACK_ENEMY_RANGE = 25;
	public final static double ENEMY_BASE_DIRECTIONAL_RANGE = 1.2;
	public final static int TRACE_POINTS = 5;
	public final static int ENEMY_RADIAL_RANGE = 8;
	// Defines parameters related to steering & throttle
	public final static int TURN_RADIUS = 50;
	public final static int TURN_ANGLE = 35;
	public final static int DRIVE_TURN_RATE = 23;
	public final static int MIN_DRIVE_SPEED = 25;
	// Defines parameters related to refueling
	public final static int REFUEL_THRESHOLD = 25;
	public final static int FUEL_FULL_THRESHOLD = 65;
	// Defines the state that the car is currently in
	private boolean isRefueling = false;
	private boolean beginRefueling = false;
	private int goalCheckpoint = 0;
	private int prevCheckpoint = -1;
	private Vec2d fuelLoc = null;
	private int stuckTicks = 0;
	private boolean tryingEscape = false;

	/**
	 * Converts the game dimension into the virtual world dimension
	 * @param dim game dimension
	 * @return virtual world dimension
	 */
	private int getBlock(int dim){
		return dim / BLOCK_SZ;
	}
	/**
	 * Converts the virtual dimension into the game world dimension
	 * @param dim virtual world dimension
	 * @return world dimension
	 */
	private int getMap(int dim){
		return dim * BLOCK_SZ;
	}

	/**
	 * Gets the location of an object on the world
	 * @param obj the object
	 * @return the game location
	 */
	private Vec2d getLocation(IObject obj){
		return new Vec2d(obj.getX(), obj.getY());
	}

	/**
	 * Converts a game location into a virtual world location
	 * @param mapLocation the game location
	 * @return virtual world location
	 */
	private Vec2d getBlockLocation(Vec2d mapLocation){
		return new Vec2d(getBlock((int)mapLocation.x), getBlock((int)mapLocation.y));
	}
	/**
	 * Converts a virtual world location into a world location
	 * @param blockLocation the virtual world location
	 * @return world location
	 */
	private Vec2d getMapLocation(Vec2d blockLocation){
		return new Vec2d(getMap((int)blockLocation.x), getMap((int)blockLocation.y));
	}

	/**
	 * Gets the value at a specified virtual world location
	 * @param pos the position
	 * @param arr the array to access
	 * @return the data
	 */
	private double getValue(Vec2d pos, double[][] arr){
		return arr[((int) pos.x)][((int) pos.y)];
	}
	/**
	 * Sets the value at a specified virtual world location
	 * @param pos the position
	 * @param arr the array to set
	 * @return the data that was set
	 */
	private double setValue(Vec2d pos, double[][] arr, double val){
		return arr[((int) pos.x)][((int) pos.y)] = val;
	}
	// Stores the virtual representation of the map
	private double[][] grid = new double[getBlock(MAX_X) + 50][getBlock(MAX_Y) + 50];
	private double[][] dist = new double[getBlock(MAX_X) + 50][getBlock(MAX_Y) + 50];
	private int[][] visit = new int[getBlock(MAX_X) + 50][getBlock(MAX_Y) + 50];
	private Vec2d[][] prev = new Vec2d[getBlock(MAX_X) + 50][getBlock(MAX_Y) + 50];

	private ArrayDeque<Integer> prevGoals = new ArrayDeque<>();
	private ArrayDeque<Integer> prevCheckpoints = new ArrayDeque<>();

	/**
	 * Sets the data contained in the grid with bound checks
	 * @param block the virtual world location
	 * @param value the value
	 */
	public void set(Vec2d block, double value){
		if(block.x < 0 || block.x >= MAX_X || block.y < 0 || block.y >= MAX_Y) return;
		grid[(int)block.x][(int)block.y] = value;
	}

	/**
	 * Synchronizes the virtual world with the game world
	 */
	public void updateMap(){
		// clear arrays
		for(double[] arr : grid){
			Arrays.fill(arr, 0);
		}
		for(double[] arr : dist){
			Arrays.fill(arr, Double.MAX_VALUE / 10);
		}
		for(int[] arr : visit){
			Arrays.fill(arr, 0);
		}
		for(Vec2d[] arr : prev){
			Arrays.fill(arr, null);
		}
		// add weighting to the edge of the map to prevent the car from bumping into it
		for(int q = 0; q < 2; q++){
			for(int i = q; i <= getBlock(MAX_X) - q; i++){
				grid[i][q] += 10 - q;
				grid[i][getBlock(MAX_Y) - q] += 10 - q;
			}
			for(int j = q; j <= getBlock(MAX_Y) - q; j++){
				grid[q][j] += 10 - q;
				grid[getBlock(MAX_X) - q][j] += 10 - q;
			}
		}

		// establish a weighted buffer zone around every opponent car
		for(ICar car : getOpponents()){
			Vec2d pos = getLocation(car);
			Vec2d block = getBlockLocation(pos);
			Vec2d dir = Vec2d.ofPolar(1, car.getHeading());
			if(car.getSpeed() < 0){
				dir = dir.multiply(-1);
			}
			// predict the trajectory of the opponent car
			ArrayList<Vec2d> rayTracer = new ArrayList<>();
			rayTracer.add(block);
			rayTracer.add(block.add(dir.multiply(3)));
			rayTracer.add(block.subtract(dir.multiply(3)));
			for(int i = 1; i <= TRACE_POINTS; i++){
				double pct = Math.abs(car.getSpeed()) * ((double)i / TRACE_POINTS) * ENEMY_BASE_DIRECTIONAL_RANGE;
				rayTracer.add(block.add(dir.multiply(pct)));
			}
			for(int x = -TRACK_ENEMY_RANGE; x <= TRACK_ENEMY_RANGE; x++){
				for(int y = -TRACK_ENEMY_RANGE; y <= TRACK_ENEMY_RANGE; y++){
					Vec2d dVec = new Vec2d(x, y);
					Vec2d nVec = dVec.add(block);
					for(int i = 0; i < rayTracer.size(); i++){
						double distance = rayTracer.get(i).distanceTo(nVec);
						if(distance <= ENEMY_RADIAL_RANGE){
							set(nVec, 500 + (ENEMY_RADIAL_RANGE - distance) * 500);
						}
					}
				}
			}
		}
	}

	/**
	 * Gets the current location of the car in terms of game space
	 * @return the location
	 */
	public Vec2d getCurrentLocation(){
		return new Vec2d(getX(), getY());
	}

	/**
	 * Gets the direction vector of the current car
	 * @return a direction vector
	 */
	public Vec2d getCurrentDirection(){
		return Vec2d.ofPolar(1, this.getHeading());
	}

	/**
	 * Checks if a position is in the game world
	 * @param loc the position
	 * @return true if it is contained in, otherwise false
	 */
	public boolean isInMap(Vec2d loc){
		return !(loc.x < 0) && !(loc.y < 0) && !(loc.x >= MAX_X) && !(loc.y >= MAX_Y);
	}

	/**
	 * Gets the predicted location of the car with respects to its current speed
	 * @return the predicted location
	 */
	public Vec2d getPredictedLocation(){
		int scalar = 1;
		if(getThrottle() < 0) scalar = -1;
		Vec2d loc = getCurrentLocation().add(getCurrentDirection().multiply(scalar * (getSpeed() + 10)));
		if(!isInMap(loc)) return getCurrentLocation();
		return loc;
	}

	// Simple offset array in all 8 directions in 2d
	private static final int[] mvarr = new int[]{1, -1, 0, 0, -1, -1, 1, 1};
	private static final int[] mvarrc = new int[]{0, 0, 1, -1, -1, 1, -1, 1};

	/**
	 * Uses a hybrid between dijkstra and grid-based breadth first search to find the lowest weight path from the current location to the destination
	 * @param dest the destination
	 * @return the most optimal path
	 */
	public CarPath getPathTo(Vec2d dest){
		for(double[] arr : dist){
			Arrays.fill(arr, Double.MAX_VALUE / 10);
		}
		for(int[] arr : visit){
			Arrays.fill(arr, 0);
		}
		PriorityQueue<Pair<Double, Vec2d>> pq = new PriorityQueue<>(new PointComparator());
		Vec2d start = getBlockLocation(getPredictedLocation());
		pq.add(new Pair<>(0d, start));
		while(pq.size() != 0){
			Pair<Double, Vec2d> v = pq.poll();
			if(v.x > getValue(v.y, dist)) continue;
			setValue(v.y, dist, v.x);
			for(int i = 0; i < 8; i++){
				Vec2d nPos = v.y.add(new Vec2d(mvarr[i], mvarrc[i]));
				if(nPos.x >= 0 && nPos.x <= (MAX_X / BLOCK_SZ) && nPos.y >= 0 && nPos.y <= (MAX_Y / BLOCK_SZ)){
					double weight = getValue(nPos, grid) + v.x + (i < 4 ? 1 : 1.4);
					if(weight < getValue(nPos, dist)){
						setValue(nPos, dist, weight);
						prev[(int)nPos.x][(int)nPos.y] = v.y;
						pq.add(new Pair<>(weight, nPos));
					}
				}
			}
		}
		Vec2d prevVal = prev[getBlock((int)dest.x)][getBlock((int)dest.y)];
		if(prevVal == null) return null;
		CarPath path = new CarPath();
		path.destination = dest;
		path.weight = getValue(prevVal, dist);
		while(prevVal != null){
			Vec2d curPt = getMapLocation(prevVal);
			visit[(int)prevVal.x][(int)prevVal.y] = 1;
			prevVal = prev[(int)prevVal.x][(int)prevVal.y];
			if(prevVal != null){
				Vec2d prevPt = getMapLocation(prevVal);
				path.length += prevPt.distanceTo(curPt);
			}
			path.points.addLast(curPt);
		}
		visit[getBlock((int)dest.x)][getBlock((int)dest.y)] = 2;

		path.points = LineUtil.simplify(3, path.points);
		for(Vec2d pt : path.points){
			visit[getBlock((int)pt.x)][getBlock((int)pt.y)] = 4;
		}
		visit[(int)start.x][(int)start.y] = 3;
		return path;
	}

	/**
	 * A function that determines the throttle of the car, this ensures the accuracy of turning
	 * @param x the distance until the next goal
	 * @param steeringMag the magnitude of the turn
	 * @return the throttle
	 */
	private double adjustedDriveThrottle(double x, double steeringMag){
		// https://www.desmos.com/calculator/qwtj7mltel
		// just a function that i made that *looks* good enough
		if(x < 50 && isRefueling) return 10;
		return Math.min(Math.max((25 + Math.log10(x + 300) * DRIVE_TURN_RATE) / (Math.pow(steeringMag, 0.5) * 0.5), MIN_DRIVE_SPEED - steeringMag), 100);
	}

	/**
	 * A function that determines how much to turn to meet an angle
	 * @param x the goal angle
	 * @return the amount to turn
	 */
	private double adjustedSteerFunction(double x){
		// change = steer * speed / 5
		// steer = change / speed * 5
		return Math.min(10, x / getSpeed() * 5);
	}

	/**
	 * Checks if it is possible to turn towards a position
	 * @param pos the position
	 * @return true if it is possible, otherwise false
	 */
	private boolean canTurnTo(Vec2d pos){
		Vec2d cur = getCurrentLocation();
		Vec2d dirVector = getCurrentDirection();
		Vec2d curPos = getCurrentLocation();
		double cHeading = dirVector.getDirection();
		double angle = curPos.getAngleTo(pos);
		double diff = Vec2d.getAngleDifference(cHeading, angle);
		if(Math.abs(diff) >= 130){
			// reverse
			cHeading = dirVector.multiply(-1).getDirection();
			angle = curPos.getAngleTo(pos);
			diff = Vec2d.getAngleDifference(cHeading, angle);
		}
		if(cur.distanceTo(pos) >= TURN_RADIUS){
			return true;
		}
		if(DEBUG) System.out.println(diff + " " + cur.distanceTo(pos));
		return Math.abs(diff) <= TURN_ANGLE;
	}

	/**
	 * Gets the angle required to reach dest from c
	 * @param dir the current heading of c
	 * @param cpos the position of c
	 * @param dest the destination position
	 * @return the angle required
	 */
	public static double getDeltaAngleTo(Vec2d dir, Vec2d cpos, Vec2d dest){
		double cHeading = dir.getDirection();
		double angle = cpos.getAngleTo(dest);
		return Vec2d.getAngleDifference(cHeading, angle);
	}

	/**
	 * Gets the angle required from the current state
	 * @param dest the destination position
	 * @return the angle required
	 */
	public double getDeltaAngleTo(Vec2d dest){
		return getDeltaAngleTo(getCurrentDirection(), getCurrentLocation(), dest);
	}

	/**
	 * Drives the car towards a certain location
	 * @param pos the position
	 * @param dist the distance to the position
	 */
	public void driveTowards(Vec2d pos, double dist){
		Vec2d dirVector = getCurrentDirection();
		Vec2d curPos = getCurrentLocation();
		double cHeading = dirVector.getDirection();
		double angle = curPos.getAngleTo(pos);
		double diff = Vec2d.getAngleDifference(cHeading, angle);
		int reverseCoeff = 1;
		if(Math.abs(diff) >= 130){
			// reverse
			cHeading = dirVector.multiply(-1).getDirection();
			angle = curPos.getAngleTo(pos);
			diff = Vec2d.getAngleDifference(cHeading, angle);
			reverseCoeff = -1;
		}

		double steerMag = Math.abs(diff);
		if(steerMag == 0) steerMag = 1;
		double steerDir = reverseCoeff * diff / steerMag;

		double realSteerAmount = adjustedSteerFunction(steerMag) * steerDir;
		setSteeringSetting((int) realSteerAmount);
		if(DEBUG) System.out.println("steer: " + realSteerAmount + " diff: " + diff + " cur: " + cHeading + " ang: " + angle);
		setThrottle(reverseCoeff * (int) adjustedDriveThrottle(dist, steerMag));
		if(DEBUG) System.out.println("thro: " + adjustedDriveThrottle(dist, steerMag) + " dist: " + dist);
	}


	/**
	 * @see com.ibm.rally.Car#initialize()
	 */
	public void initialize() {
		// put implementation here
		totalCheckpoints = World.getCheckpoints().length;
	}

	/**
	 * Gets the optimal action that the car should be executing
	 * @return
	 */
	public CarPath getOptimalGoal(){
		IObject[] checkpoints = World.getCheckpoints();
		IObject[] fuelDepots = World.getFuelDepots();

		Vec2d currentPos = getCurrentLocation();

		if(isRefueling){
			double dist = currentPos.distanceTo(fuelLoc);
			if(getFuel() >= FUEL_FULL_THRESHOLD){
				isRefueling = false;
			}else{
				if(dist > 25){
					// somehow the car is out of the fuel depot's range
					driveTowards(fuelLoc, dist);
				}else{
					setThrottle(0);
				}
				return null;
			}
		}
		if(getFuel() <= REFUEL_THRESHOLD || beginRefueling){
			beginRefueling = true;
			if(DEBUG) System.out.println("refueling");
			CarPath bestDepot = null;
			for(IObject depot : fuelDepots){
				Vec2d pos = getLocation(depot);
				CarPath path = getPathTo(pos);
				if(path == null || !canTurnTo(pos)) continue;
				path.destination = pos;
				if(bestDepot == null || path.weight < bestDepot.weight){
					bestDepot = path;
				}
			}
			double dist = currentPos.distanceTo(bestDepot.destination);
			if(dist <= Math.abs(getSpeed()) || dist <= 55){
				beginRefueling = false;
				isRefueling = true;
				fuelLoc = bestDepot.destination;
				setThrottle(0);
				return null;
			}
			return bestDepot;
		}

		double bestCheckpoint = Double.MAX_VALUE;
		CarPath bestCheckpointPath = null;
		int goingto = -1;
		for(int i = 0; i < checkpoints.length; i++){
			IObject chk = checkpoints[i];
			Vec2d pos = getLocation(chk);
			if(!prevGoals.isEmpty() && !prevGoals.getLast().equals(i) && prevGoals.contains(i)) continue;
			if(prevCheckpoints.contains(i)) continue;
			if(!canTurnTo(pos)) continue;
			CarPath path = getPathTo(pos);
			if(path == null) continue;
			double weight = i == goalCheckpoint ? -900 : 0;
			weight += path.weight;
			if(weight <= bestCheckpoint){
				bestCheckpoint = weight;
				bestCheckpointPath = path;
				goingto = i;
			}
		}
		if(prevGoals.isEmpty() || !prevGoals.getLast().equals(goalCheckpoint))
			prevGoals.add(goingto);
		while(prevGoals.size() >= 4) prevGoals.removeFirst();
		if (bestCheckpointPath != null) {
			if(DEBUG) System.out.println(bestCheckpointPath.destination + " " + goingto + " " + goalCheckpoint + " " + getPreviousCheckpoint());
		}
		return bestCheckpointPath;
	}

	/**
	 * @see com.ibm.rally.Car#move(int, boolean, ICar, ICar)
	 */
	public void move(int lastMoveTime, boolean hitWall, ICar collidedWithCar, ICar hitBySpareTire) {
		// put implementation here
		if(!isRefueling && Math.abs(getSpeed()) <= 0.1){
			stuckTicks++;
		}else{
			stuckTicks = 0;
			tryingEscape = false;
		}
		// prevent car from getting stuck in one place
		if(stuckTicks >= 20){
			tryingEscape = true;
			stuckTicks = 5;
		}
		if(tryingEscape){
			// try to wiggle the car out
			enterProtectMode();
			setThrottle((int) (Math.random() * 200 - 100));
			setSteeringSetting((int) (Math.random() * 20 - 10));
			stuckTicks--;
			if(stuckTicks <= 0) {
				tryingEscape = false;
				stuckTicks = 0;
			}else{
				return;
			}
		}
		updateMap();
		if(getPreviousCheckpoint() != prevCheckpoint){
			prevCheckpoints.addLast(goalCheckpoint);
			prevCheckpoint = getPreviousCheckpoint();
			prevCheckpoints.addLast(prevCheckpoint);
			while(prevCheckpoints.size() >= 4) prevCheckpoints.removeFirst();
			goalCheckpoint = (getPreviousCheckpoint() + 1) % totalCheckpoints;
		}

		if(getValue(getBlockLocation(getCurrentLocation()), grid) >= 5 && !isRefueling){
			if(!this.isInProtectMode()){
				// protect! an unwanted object is near us
				enterProtectMode();
			}
		}

		for(IObject tire : World.getSpareTiresOnTrack()){
			if(getCurrentLocation().distanceTo(getLocation(tire)) <= 100){
				// protect! an unwanted object is near us
				enterProtectMode();
			}
		}

		// if there is a good candidate, throw the tire
		for(ICar car : getOpponents()){
			if(Math.abs(getDeltaAngleTo(getLocation(car))) <= 10 && isReadyToThrowSpareTire() && !car.isInProtectMode()){
				throwSpareTire();
			}
		}

		CarPath path = getOptimalGoal();
		if(path == null) return;
		driveTowards(path.points.peekFirst(), path.length);
		if(DEBUG) System.out.println(getSpeed());
	}
}