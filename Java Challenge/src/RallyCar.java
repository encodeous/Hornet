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
	 * Converts screen space coordinates to cartesian
	 * @return
	 */
	public Vec2d toCartesian(){
		return new Vec2d(-y, x);
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
	public double length = 0;
	public double weight = 0;
	public Vec2d destination;

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

class PointComparator implements Comparator<Pair<Double, Vec2d>>{

	@Override
	public int compare(Pair<Double, Vec2d> o1, Pair<Double, Vec2d> o2) {
		if(o1.x.equals(o2.x)){
			return o1.y.compareTo(o2.y);
		}
		return Double.compare(o1.x, o2.x);
	}
}

class RdpSimplifier{
	public static double getDist(Vec2d ep1, Vec2d ep2, Vec2d pt){
		// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		double dx1 = ep2.x - ep1.x;
		double dy1 = ep2.y - ep1.y;
		return Math.abs(dx1 * (ep1.y - pt.y) - (ep1.x - pt.x) * (dy1)) / Math.sqrt(dx1 * dx1 + dy1 * dy1);
	}

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
				double d = getDist(p1, p2, point);
				if(d > md){
					md = d;
					mdi = idx;
				}
			}
			idx++;
		}
		if(md < epsilon || input.size() == 2){
			ArrayDeque<Vec2d> deq = new ArrayDeque<Vec2d>();
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
		ArrayDeque<Vec2d> ans = new ArrayDeque<Vec2d>();
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
@JavaChallenge(name="Hornet", organization="Bees")
public class RallyCar extends Car {
	/**
	 * @see com.ibm.rally.Car#getColor()
	 */
	public byte getColor() {
		return CAR_YELLOW;
	}

	private int totalCheckpoints;
	private int goalCheckpoint = 0;
	private int prevCheckpoint = -1;
	private int targetCheckpoint = 0;
	public final static int MAX_X = 1010;
	public final static int MAX_Y = 580;
	public final static int BLOCK_SZ = 10;
	public final static int ENEMY_RANGE = 11;
	private int getBlock(int dim){
		return dim / BLOCK_SZ;
	}
	private int getMap(int dim){
		return dim * BLOCK_SZ;
	}
	private Vec2d getLocation(IObject obj){
		return new Vec2d(obj.getX(), obj.getY());
	}
	private Vec2d getBlockLocation(Vec2d mapLocation){
		return new Vec2d(getBlock((int)mapLocation.x), getBlock((int)mapLocation.y));
	}
	private Vec2d getMapLocation(Vec2d blockLocation){
		return new Vec2d(getMap((int)blockLocation.x), getMap((int)blockLocation.y));
	}
	private double getValue(Vec2d pos, double[][] arr){
		return arr[((int) pos.x)][((int) pos.y)];
	}
	private double setValue(Vec2d pos, double[][] arr, double val){
		return arr[((int) pos.x)][((int) pos.y)] = val;
	}
	private double[][] grid = new double[getBlock(MAX_X) + 50][getBlock(MAX_Y) + 50];
	private double[][] dist = new double[getBlock(MAX_X) + 50][getBlock(MAX_Y) + 50];
	private int[][] visit = new int[getBlock(MAX_X) + 50][getBlock(MAX_Y) + 50];
	private Vec2d[][] prev = new Vec2d[getBlock(MAX_X) + 50][getBlock(MAX_Y) + 50];

	private ArrayDeque<Integer> prevCheckpoints = new ArrayDeque<>();
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
			Arrays.fill(arr, Double.MAX_VALUE / 10);
		}
		for(int[] arr : visit){
			Arrays.fill(arr, 0);
		}
		for(Vec2d[] arr : prev){
			Arrays.fill(arr, null);
		}
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

		for(ICar car : getOpponents()){
			Vec2d pos = getLocation(car);
			Vec2d block = getBlockLocation(pos);
			for(int x = -ENEMY_RANGE; x <= ENEMY_RANGE; x++){
				for(int y = -ENEMY_RANGE; y <= ENEMY_RANGE; y++){
					Vec2d dVec = new Vec2d(x, y);
					Vec2d nVec = dVec.add(block);
					double distance = nVec.distanceTo(block);
					if(distance <= ENEMY_RANGE){
						set(nVec, get(nVec) + 500 + (ENEMY_RANGE - distance) * 500);
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

	public boolean isInMap(Vec2d loc){
		return !(loc.x < 0) && !(loc.y < 0) && !(loc.x >= MAX_X) && !(loc.y >= MAX_Y);
	}

	public Vec2d getPredictedLocation(){
		int scalar = 1;
		if(getThrottle() < 0) scalar = -1;
		Vec2d loc = getCurrentLocation().add(getCurrentDirection().multiply(scalar * (getSpeed() + 10)));
		if(!isInMap(loc)) return getCurrentLocation();
		return loc;
	}

	private static final int[] mvarr = new int[]{1, -1, 0, 0, -1, -1, 1, 1};
	private static final int[] mvarrc = new int[]{0, 0, 1, -1, -1, 1, -1, 1};
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

		path.points = RdpSimplifier.simplify(15, path.points);
		for(Vec2d pt : path.points){
			visit[getBlock((int)pt.x)][getBlock((int)pt.y)] = 4;
		}
		visit[(int)start.x][(int)start.y] = 3;
		return path;
	}

	private double adjustedDriveThrottle(double x, double steeringMag){
		// https://www.desmos.com/calculator/gflge44xrw
		// just a function that i made that *looks* good enough
		return Math.min(Math.max((25 + Math.log10(x + 300) * 23) / (Math.pow(steeringMag, 0.5) * 0.5), 10), 100);
	}

	private double adjustedSteerFunction(double x){
		// https://www.desmos.com/calculator/8ognwcjnjf
		// just another function that i made that *looks* good enough
		return (Math.pow(x * 40, 0.4) - 1 / 20.0 * x) / 3.0;
	}

	public void driveTowards(Vec2d pos, double dist){
		Vec2d dirVector = getCurrentDirection();
		Vec2d curPos = getCurrentLocation();
		double cHeading = dirVector.getDirection();
		double angle = curPos.getAngleTo(pos);
		double diff = Vec2d.getAngleDifference(cHeading, angle);
		int reverseCoeff = 1;
		if(Math.abs(diff) >= 110){
			// reverse
			cHeading = dirVector.multiply(-1).getDirection();
			angle = curPos.getAngleTo(pos);
			diff = Vec2d.getAngleDifference(cHeading, angle);
			reverseCoeff = -1;
		}

		double steerMag = Math.abs(diff);
		double steerDir = reverseCoeff * diff / steerMag;

		double realSteerAmount = adjustedSteerFunction(steerMag) * steerDir;
		setSteeringSetting((int) realSteerAmount);
//		System.out.println("steer: " + realSteerAmount + " diff: " + diff + " cur: " + cHeading + " ang: " + angle);
		setThrottle(reverseCoeff * (int) adjustedDriveThrottle(dist, steerMag));
//		System.out.println("thro: " + adjustedDriveThrottle(dist, steerMag) + " dist: " + dist);
	}


	/**
	 * @see com.ibm.rally.Car#initialize()
	 */
	public void initialize() {
		// put implementation here
		totalCheckpoints = World.getCheckpoints().length;
	}

	private boolean isRefueling = false;

	public CarPath getOptimalGoal(){
		IObject[] checkpoints = World.getCheckpoints();
		IObject[] fuelDepots = World.getFuelDepots();
		IObject[] tireDepots = World.getSpareTireDepots();

		Vec2d currentPos = getCurrentLocation();

		if(isRefueling){
			if(getFuel() >= 70){
				isRefueling = false;
			}else{
				return null;
			}
		}
		if(getFuel() <= 25){
			System.out.println("refueling");
			CarPath bestDepot = null;
			for(IObject depot : fuelDepots){
				Vec2d pos = getLocation(depot);
				CarPath path = getPathTo(pos);
				if(path == null) continue;
				if(bestDepot == null || path.weight < bestDepot.weight){
					bestDepot = path;
				}
			}
			double dist = currentPos.distanceTo(bestDepot.destination);
			if(dist <= 52){
				isRefueling = true;
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
			if(prevCheckpoints.contains(i)) continue;
			CarPath path = getPathTo(pos);
			if(path == null) continue;
			double weight = i == goalCheckpoint ? -400 : 0;
			weight += path.weight;
			if(weight <= bestCheckpoint){
				bestCheckpoint = weight;
				bestCheckpointPath = path;
				goingto = i;
			}
		}
		if (bestCheckpointPath != null) {
			System.out.println(bestCheckpointPath.destination + " " + goingto + " " + goalCheckpoint + " " + getPreviousCheckpoint());
		}
		return bestCheckpointPath;
	}

	/**
	 * @see com.ibm.rally.Car#move(int, boolean, ICar, ICar)
	 */
	public void move(int lastMoveTime, boolean hitWall, ICar collidedWithCar, ICar hitBySpareTire) {
		// put implementation here

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

		CarPath path = getOptimalGoal();
		if(path == null) return;
		driveTowards(path.points.peekFirst(), path.length);
		System.out.println(getSpeed());
//		for(int i = 0; i <= MAX_Y / BLOCK_SZ; i++){
//			for(int j = 0; j <= MAX_X / BLOCK_SZ; j++){
////				if(visit[j][i] == 1){
////					System.out.print("X");
////				}
//				if(visit[j][i] == 4){
//					System.out.print("X");
//				}
//				if(visit[j][i] == 2){
//					System.out.print("D");
//				}
//				if(visit[j][i] == 3){
//					System.out.print("S");
//				}
//				else{
//					System.out.print(".");
//				}
//			}
//			System.out.println();
//		}
	}
}