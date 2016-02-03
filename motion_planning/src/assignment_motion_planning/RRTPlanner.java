/**
 * This algorithm is from the following paper
 * Steven M. LaValle and James  Kuffner Jr.,
 * Randomized Kinodynamic Planning, 
 * The International Journal of Robotics Research 20 (5), pp. 378–400.
 * http://dx.doi.org/10.1177/02783640122067453
 */

package assignment_motion_planning;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import javafx.geometry.Point2D;
import javafx.util.Pair;

public class RRTPlanner extends MotionPlanner {
    private static final double DEFAULT_DELTA = 0.1;  // Duration for the control
    private Map<Vector, Edge> map; 
    private boolean ALTER_RRT = false;
    /**
     * Constructor 
     * @param environment the workspace
     * @param robot       the robot
     */
    public RRTPlanner(Environment environment, Robot robot) {
        super(environment, robot);
        setup();
    }
    
    @Override
    public List<Pair<Point2D, Point2D>>getEdges() {
        // make an edge list ArrayList
    	List<Pair<Point2D, Point2D>> edgeList = new ArrayList<Pair<Point2D, Point2D>>();
    	
    	// for every node in keyset 
    	for(Vector node : map.keySet()){
    		// GET THE EDGE, AND IF IT EXISTS
    		if(map.get(node) == null){continue;}
    				Point2D first = new Point2D(map.get(node).getEnd().get(0), map.get(node).getEnd().get(1));
    				Point2D second = new Point2D(map.get(node).getStart().get(0), map.get(node).getStart().get(1));
    				Pair<Point2D, Point2D> edgePair = new Pair(first, second);
    				edgeList.add(edgePair);

    	}
    			return edgeList;
    }
    
    @Override
    public int getSize() {
        // YOU WILL WRITE THIS METHOD
        return map.size();
    }

    @Override
    protected void setup() {
        map = new HashMap<Vector, Edge>();
        map.put(getStart(), null);
    }
    
    @Override
    protected void growMap(int K) {
        
    	int step = 0;
    	for(int i = 0; i < K; i++){

    		// find nearestNeighbor
    		Vector config = getRobot().getRandomConfiguration(getEnvironment(), random);

    		if(ALTER_RRT){
    			ArrayList<Vector> keys = new ArrayList<Vector>(map.keySet());
    			Random ran = new Random();
    			if(step < 0 | step > keys.size()-1){
    				step = 0;
    			}
    		
    			int n = ran.nextInt(keys.size() - step) + step;
    			Vector qnear =  keys.get(n);
    			// invoke newConf
    		
    			newConf(qnear, .9);
    			step += 199*(keys.size()/200);
    		 }
    		else {
        	Vector qnear = nearestNeighbor(map.keySet(), config);
      		newConf(qnear, DEFAULT_DELTA);
    			
    		}
    		
    	}
    	// findpath
    }

    /**
     * Generate a new configuration from a configuration and insert it
     * @param qnear    the beginning configuration of the random motion
     * @param duration the duration of the random motion
     * @return true if one new configuration is inserted, and false otherwise
     */
    @SuppressWarnings("boxing")
    private boolean newConf(Vector qnear, double duration) {
    	// generate a random config   	
        // get random control 	
    		// use Robot.move
    		Vector con = getRobot().getRandomControl(random);
    		
    		Vector newMove = getRobot().move(qnear, con, duration);
    		
    		Trajectory newTraj = new Trajectory(con, duration);
    		
    		
    		if(getEnvironment().isValidMotion(getRobot(), newMove, newTraj, RESOLUTION)){
    			// if is Valid, then add to map
    			if(!map.containsKey(newMove)){
    				
    				Edge newEdge = new Edge(newMove, qnear, duration, con);
    				map.put(newMove, newEdge);
    				return true;
    				
    			}
    			
    		}
        return false;
    }
    
    @SuppressWarnings("boxing")
    @Override
    protected Trajectory findPath() {
        // backchain up path and return a trajectory
    		Trajectory path = new Trajectory();
    		List<Vector> tempCon = new ArrayList<Vector>();
    		List<Double> tempDur = new ArrayList<Double>();
    		
    		Vector cur = nearestNeighbor(map.keySet(), getGoal());
 		
    	
    		
    		while(map.get(cur) !=null ){
    		
    			tempCon.add(map.get(cur).getControl());
    			tempDur.add(map.get(cur).getDur());
    			cur = map.get(cur).getEnd();
    		
    		}

    		// reverse lists and 
    		for(int i = tempCon.size()-1; i >=0; i--){
    			path.addControl(tempCon.get(i), tempDur.get(i));
       	}
    	
        return path;
    }

    @Override
    protected void reset() {
    		map.clear();
    }

}
