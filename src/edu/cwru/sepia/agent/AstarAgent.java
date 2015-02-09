package edu.cwru.sepia.agent;

import edu.cwru.sepia.action.Action;
import edu.cwru.sepia.agent.AstarAgent.MapLocation;
import edu.cwru.sepia.environment.model.history.History;
import edu.cwru.sepia.environment.model.state.ResourceNode;
import edu.cwru.sepia.environment.model.state.State;
import edu.cwru.sepia.environment.model.state.Unit;
import edu.cwru.sepia.util.Direction;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

public class AstarAgent extends Agent {

    class MapLocation implements Comparable<MapLocation>
    {
        public int x, y;
        public MapLocation cameFrom;
        public double f_score;
        public double g_score;

        public MapLocation(int x, int y, MapLocation cameFrom, float cost)
        {
            this.x = x;
            this.y = y;
            this.cameFrom = cameFrom;
        }
        
        public MapLocation(int x, int y, MapLocation cameFrom, float cost, double f, double g)
        {
            this(x, y, cameFrom, cost);
            f_score = f;
            g_score = g;   
        }
        
        @Override
        public String toString()
        {
        	return "("+x+","+y+")";
        }

		@Override
		public int compareTo(MapLocation o) {
			return (int) (this.f_score - o.f_score);
		}
		
		@Override
		public int hashCode(){
			return Integer.valueOf(x).hashCode() + Integer.valueOf(x).hashCode()^23;
		}
		
		@Override
		public boolean equals(Object o){
			if(o instanceof MapLocation){
				MapLocation o_ml = (MapLocation) o;
				return this.x == o_ml.x && this.y == o_ml.y;
			}else{
				return false;
			}
		}
    }

    Stack<MapLocation> path;
    int footmanID, townhallID, enemyFootmanID;
    MapLocation nextLoc;

    private long totalPlanTime = 0; // nsecs
    private long totalExecutionTime = 0; //nsecs
    
    private boolean killedTownHall;

    public AstarAgent(int playernum)
    {
        super(playernum);
        killedTownHall = false;
        System.out.println("Constructed AstarAgent");
    }

    @Override
    public Map<Integer, Action> initialStep(State.StateView newstate, History.HistoryView statehistory) {
        // get the footman location
        List<Integer> unitIDs = newstate.getUnitIds(playernum);

        if(unitIDs.size() == 0)
        {
            System.err.println("No units found!");
            return null;
        }

        footmanID = unitIDs.get(0);

        // double check that this is a footman
        if(!newstate.getUnit(footmanID).getTemplateView().getName().equals("Footman"))
        {
            System.err.println("Footman unit not found");
            return null;
        }

        // find the enemy playernum
        Integer[] playerNums = newstate.getPlayerNumbers();
        int enemyPlayerNum = -1;
        for(Integer playerNum : playerNums)
        {
            if(playerNum != playernum) {
                enemyPlayerNum = playerNum;
                break;
            }
        }

        if(enemyPlayerNum == -1)
        {
            System.err.println("Failed to get enemy playernumber");
            return null;
        }

        // find the townhall ID
        List<Integer> enemyUnitIDs = newstate.getUnitIds(enemyPlayerNum);

        if(enemyUnitIDs.size() == 0)
        {
            System.err.println("Failed to find enemy units");
            return null;
        }

        townhallID = -1;
        enemyFootmanID = -1;
        for(Integer unitID : enemyUnitIDs)
        {
            Unit.UnitView tempUnit = newstate.getUnit(unitID);
            String unitType = tempUnit.getTemplateView().getName().toLowerCase();
            if(unitType.equals("townhall"))
            {
                townhallID = unitID;
            }
            else if(unitType.equals("footman"))
            {
                enemyFootmanID = unitID;
            }
            else
            {
                System.err.println("Unknown unit type");
            }
        }

        if(townhallID == -1) {
            System.err.println("Error: Couldn't find townhall");
            return null;
        }

        long startTime = System.nanoTime();
        path = findPath(newstate);
        totalPlanTime += System.nanoTime() - startTime;
        
        
        Map<Integer, Action> ret = middleStep(newstate, statehistory);
        //System.out.println("ret:"+ret);
        return ret;
    }

    @Override
    public Map<Integer, Action> middleStep(State.StateView newstate, History.HistoryView statehistory) {
       /* if(killedTownHall){
        	System.out.println("Townhall is dead");
        	return new HashMap<Integer, Action>();
        	
        }*/
    	long startTime = System.nanoTime();
        long planTime = 0;

        Map<Integer, Action> actions = new HashMap<Integer, Action>();
        
        
        if(enemyFootmanID !=-1 && shouldReplanPath(newstate, statehistory, path)) {
            long planStartTime = System.nanoTime();
            path = findPath(newstate);
            planTime = System.nanoTime() - planStartTime;
            totalPlanTime += planTime;
        }

        Unit.UnitView footmanUnit = newstate.getUnit(footmanID);

        int footmanX = footmanUnit.getXPosition();
        int footmanY = footmanUnit.getYPosition();

        if(!path.empty() && (nextLoc == null || (footmanX == nextLoc.x && footmanY == nextLoc.y))) {

            // stat moving to the next step in the path
            nextLoc = path.pop();

            System.out.println("Moving to (" + nextLoc.x + ", " + nextLoc.y + ")");
        }

        if(nextLoc != null && (footmanX != nextLoc.x || footmanY != nextLoc.y))
        {
            int xDiff = nextLoc.x - footmanX;
            int yDiff = nextLoc.y - footmanY;

            // figure out the direction the footman needs to move in
            Direction nextDirection = getNextDirection(xDiff, yDiff);

            actions.put(footmanID, Action.createPrimitiveMove(footmanID, nextDirection));
        } else {
            Unit.UnitView townhallUnit = newstate.getUnit(townhallID);

            // if townhall was destroyed on the last turn
            if(townhallUnit == null) {
            	killedTownHall = true;
                terminalStep(newstate, statehistory);
                return actions;
            }

            if(Math.abs(footmanX - townhallUnit.getXPosition()) > 1 ||
                    Math.abs(footmanY - townhallUnit.getYPosition()) > 1)
            {
                System.err.println("Invalid plan. Cannot attack townhall");
                totalExecutionTime += System.nanoTime() - startTime - planTime;
                return actions;
            }
            else {
                System.out.println("Attacking TownHall");
                // if no more movements in the planned path then attack
                actions.put(footmanID, Action.createPrimitiveAttack(footmanID, townhallID));
            }
        }

        totalExecutionTime += System.nanoTime() - startTime - planTime;
        return actions;
    }

    @Override
    public void terminalStep(State.StateView newstate, History.HistoryView statehistory) {
        
    	System.out.println("Total turns: " + newstate.getTurnNumber());
        System.out.println("Total planning time: " + totalPlanTime/1e9);
        System.out.println("Total execution time: " + totalExecutionTime/1e9);
        System.out.println("Total time: " + (totalExecutionTime + totalPlanTime)/1e9);
    }

    @Override
    public void savePlayerData(OutputStream os) {

    }

    @Override
    public void loadPlayerData(InputStream is) {

    }

    /**
     * You will implement this method.
     *
     * This method should return true when the path needs to be replanned
     * and false otherwise. This will be necessary on the dynamic map where the
     * footman will move to block your unit.
     *
     * @param state
     * @param history
     * @param currentPath
     * @return
     */
    private boolean shouldReplanPath(State.StateView state, History.HistoryView history, Stack<MapLocation> currentPath)
    {
    	if(state.getUnit(enemyFootmanID) == null) return false;
    	
    	Unit.UnitView footmanUnit = state.getUnit(footmanID);
    	Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
    	
        int footmanX = footmanUnit.getXPosition();
        int footmanY = footmanUnit.getYPosition();

        int enemyFootmanX = enemyFootmanUnit.getXPosition();
        int enemyFootmanY = enemyFootmanUnit.getYPosition();
        
        /*
        if(Math.abs(footmanX-enemyFootmanX)+Math.abs(footmanY-enemyFootmanY)<=2)
        	return true;
        */
        
        Vector<MapLocation> v = currentPath;
        //System.out.print(v.toString());
        int count = 0;
        for(MapLocation m : v)
        {
        	if(m.x == enemyFootmanX && m.y == enemyFootmanY)
        	{
        		return true;
        	}
        	if(count == 5 ) break;
        	count++;
        }
        
        return false;
    }

    /**
     * This method is implemented for you. You should look at it to see examples of
     * how to find units and resources in Sepia.
     *
     * @param state
     * @return
     */
    private Stack<MapLocation> findPath(State.StateView state)
    {
        Unit.UnitView townhallUnit = state.getUnit(townhallID);
        Unit.UnitView footmanUnit = state.getUnit(footmanID);

        MapLocation startLoc = new MapLocation(footmanUnit.getXPosition(), footmanUnit.getYPosition(), null, 0);

        MapLocation goalLoc = new MapLocation(townhallUnit.getXPosition(), townhallUnit.getYPosition(), null, 0);

        MapLocation footmanLoc = null;
        if(enemyFootmanID != -1) {
            Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
            footmanLoc = new MapLocation(enemyFootmanUnit.getXPosition(), enemyFootmanUnit.getYPosition(), null, 0);
        }

        // get resource locations
        List<Integer> resourceIDs = state.getAllResourceIds();
        Set<MapLocation> resourceLocations = new HashSet<MapLocation>();
        for(Integer resourceID : resourceIDs)
        {
            ResourceNode.ResourceView resource = state.getResourceNode(resourceID);

            resourceLocations.add(new MapLocation(resource.getXPosition(), resource.getYPosition(), null, 0));
        }

        return AstarSearch(startLoc, goalLoc, state.getXExtent(), state.getYExtent(), footmanLoc, resourceLocations);
    }
    /**
     * This is the method you will implement for the assignment. Your implementation
     * will use the A* algorithm to compute the optimum path from the start position to
     * a position adjacent to the goal position.
     *
     * You will return a Stack of positions with the top of the stack being the first space to move to
     * and the bottom of the stack being the last space to move to. If there is no path to the townhall
     * then return null from the method and the agent will print a message and do nothing.
     * The code to execute the plan is provided for you in the middleStep method.
     *
     * As an example consider the following simple map
     *
     * F - - - -
     * x x x - x
     * H - - - -
     *
     * F is the footman
     * H is the townhall
     * x's are occupied spaces
     *
     * xExtent would be 5 for this map with valid X coordinates in the range of [0, 4]
     * x=0 is the left most column and x=4 is the right most column
     *
     * yExtent would be 3 for this map with valid Y coordinates in the range of [0, 2]
     * y=0 is the top most row and y=2 is the bottom most row
     *
     * resourceLocations would be {(0,1), (1,1), (2,1), (4,1)}
     *
     * The path would be
     *
     * (1,0)
     * (2,0)
     * (3,1)
     * (2,2)
     * (1,2)
     *
     * Notice how the initial footman position and the townhall position are not included in the path stack
     *
     * @param start Starting position of the footman
     * @param goal MapLocation of the townhall
     * @param xExtent Width of the map
     * @param yExtent Height of the map
     * @param resourceLocations Set of positions occupied by resources
     * @return Stack of positions with top of stack being first move in plan
     */
    private Stack<MapLocation> AstarSearch(MapLocation start, MapLocation goal, int xExtent, int yExtent, MapLocation enemyFootmanLoc, Set<MapLocation> resourceLocations)
    {
    	//Initialize the open and closed sets
    	Set<MapLocation> closedSet = new HashSet<MapLocation>();
    	//Set<MapLocation> openSet = new HashSet<MapLocation>();
    	PriorityQueue<MapLocation> openSet = new PriorityQueue<MapLocation>();
    	MapLocation s = new MapLocation(start.x, start.y, start.cameFrom, 0, hfun(start, goal), 0);
    	openSet.add(s);
    	//Map<MapLocation,MapLocation> cameFrom = new HashMap<MapLocation,MapLocation>();
    	
    	//Map<MapLocation,Double> g_score = new HashMap<MapLocation,Double>();
    	//Map<MapLocation,Double> h_score = new HashMap<MapLocation,Double>();
    	//Map<MapLocation,Double> f_score = new HashMap<MapLocation,Double>();
    	
    	//g_score.put(start, 0.0);
    	//h_score.put(start, hfun(start,goal));
    	//f= g+h;
    	//f_score.put(start, hfun(start,goal));
    	
    	MapLocation current;
    	double currentF;
    	double currentG;
    	
    	while(!openSet.isEmpty())
    	{
    		//System.out.println(openSet.size());
    		//pop off the current value
    		current = openSet.remove();
    		//System.out.println("("+current.x+" , "+current.y+")");
    		if(current.equals(goal))
    		{
    			System.out.println("done");
    			return reconstructPath(start,current);
    		}
    		
    		//openSet.remove(current);
    		closedSet.add(current);
    		Set<MapLocation> sucessors = getSucessors(current, enemyFootmanLoc, resourceLocations, xExtent, yExtent);
    		
    		//System.out.println(sucessors.size());
    		
    		for(MapLocation neighbor : sucessors)
    		{
    			//if(setContains(closedSet,neighbor.x,neighbor.y))
    			if(closedSet.contains(neighbor))
    				continue;
    			double g_score_estimate = current.g_score + dist_between(current,neighbor);
    			
    			/*double g_neighbor;
    			if(g_score.get(neighbor)==null)
    				g_neighbor = 0;
    			else
    				g_neighbor = g_score.get(neighbor);
    			*/
    			//if(!setContains(openSet,neighbor.x,neighbor.y) )|| g_score_estimate < g_neighbor)
    			//{
    				//cameFrom.put(neighbor, current);
    				//g_score.put(neighbor, g_score_estimate);
    				//h_score.put(neighbor, hfun(neighbor,goal));
    				//f_score.put(neighbor, g_score.get(neighbor)+h_score.get(neighbor));
    				//if(!setContains(openSet,neighbor.x,neighbor.y)){
    				if(openSet.contains(neighbor)){
    					MapLocation n = new MapLocation(neighbor.x, neighbor.y, current, 0, g_score_estimate + hfun(neighbor, goal), g_score_estimate);
    					openSet.add(n);
    					
    				}
    			//}
    			
    		}
    	}
        // there is no path
    	System.out.println("No Available Path");
        return new Stack<MapLocation>();
    }
 
    
    /*
     * rebuilds the path by taking the parent value of the agent and backtracks all the way up.
     */
    private Stack<MapLocation> reconstructPath(MapLocation start, MapLocation end)
    {
    	Stack<MapLocation> toReturn = new Stack<MapLocation>();
    	//toReturn.push(end);
    	MapLocation current = end;
    	while(current!=start)
    	{
    		current = current.cameFrom;
    		toReturn.push(current);
    		//System.out.println(toReturn.toString());
    	}
    	toReturn.pop();
    	//System.out.println(toReturn.toString());
    	return toReturn;
    	
    }
    
    /*
     * returns the set of locations we can move to from where the agent is currently located
     */
    private Set<MapLocation> getSucessors(MapLocation current, MapLocation enemyFootmanLoc, Set<MapLocation> resourceLocations, int xExtent, int yExtent)
    {
    	int xval = current.x;
    	int yval = current.y;
    	Set<MapLocation> toReturn = new HashSet<MapLocation>();
    	for(int i = -1; i<=1; i++)
    	{
    		for(int j = -1; j<=1; j++)
    		{
    			MapLocation loc = new MapLocation(xval+i,yval+j,current,0);
    			//checks if valid and that we are not at the current location
    			if(isValidLocation(current, loc, enemyFootmanLoc, resourceLocations, xExtent,  yExtent))
    			{
    				toReturn.add(loc);
    				//System.out.println("("+loc.x+" , "+loc.y+")");
    			}
    		}
    	}
    	return toReturn;
    	
    }
    
    /*
     * works so that we will move horizontally when its worth it,
     * and makes diagonal at a marginal cost higher than going just up down left or right
     */
    private double dist_between(MapLocation current, MapLocation neighbor)
    {
    	return 1.0;
    }
    
    /*
     * checks if the location is a location we can move to
     * not updated for the dynamic case yet
     */
    private boolean isValidLocation(MapLocation current, MapLocation loc, MapLocation enemyFootmanLoc, Set<MapLocation> resourceLocations, int xExtent, int yExtent)
    {
    	int xval = loc.x;
    	int yval = loc.y;
      	
    	if(xval<0 || yval<0 || xval>=xExtent || yval>=yExtent)
    		return false;
    	else if(current.x==loc.x && current.y==loc.y)
    		return false;
    	else if(enemyFootmanLoc!= null && current.x==enemyFootmanLoc.x && current.y==enemyFootmanLoc.y)
    		return false;
    	else if (setContains(resourceLocations, xval,yval))
    		return false;
    	else
    		return true;
    }
    
    /*
     *checks if a set contains the specific map location values in the x and y coordinates
     *this method is done because java native methods don't work properly with these objects 
     *
     */
    private boolean setContains(Set<MapLocation> set, int x, int y)
    {
    	for(MapLocation m : set)
    	{
    		if(m.x == x && m.y == y)
    			return true;
    	}
    	return false;
    }
    
    /*
     * compares values in the open set and their heuristic values
     *  and chooses the lowest value based on what direction we want to go 
     */
    private MapLocation getLowestF(Set<MapLocation> oSet, Map<MapLocation,Double> f)
    {
    	MapLocation loc = null; 
    	double val = Integer.MAX_VALUE;
    	for(MapLocation m:oSet)
    	{
    		if(f.get(m)<=val)
    		{
    			loc = m;
    			val = f.get(loc);
    		}
    	}
    	return loc;
    }
    /*
     * the huristic function used for A* Search using the Chebyshev distance huristic
     * 
     * @param start Map Location where we are starting huristic calculation from
     * @param goal Map Location of the goal
     */
    private Double hfun(MapLocation start, MapLocation goal)
    {
    	return Math.max((double)(Math.abs(start.x - goal.x)),(double)(Math.abs(start.y - goal.y)));
    }
    /**
     * Primitive actions take a direction (e.g. NORTH, NORTHEAST, etc)
     * This converts the difference between the current position and the
     * desired position to a direction.
     *
     * @param xDiff Integer equal to 1, 0 or -1
     * @param yDiff Integer equal to 1, 0 or -1
     * @return A Direction instance (e.g. SOUTHWEST) or null in the case of error
     */
    private Direction getNextDirection(int xDiff, int yDiff) {

        // figure out the direction the footman needs to move in
        if(xDiff == 1 && yDiff == 1)
        {
            return Direction.SOUTHEAST;
        }
        else if(xDiff == 1 && yDiff == 0)
        {
            return Direction.EAST;
        }
        else if(xDiff == 1 && yDiff == -1)
        {
            return Direction.NORTHEAST;
        }
        else if(xDiff == 0 && yDiff == 1)
        {
            return Direction.SOUTH;
        }
        else if(xDiff == 0 && yDiff == -1)
        {
            return Direction.NORTH;
        }
        else if(xDiff == -1 && yDiff == 1)
        {
            return Direction.SOUTHWEST;
        }
        else if(xDiff == -1 && yDiff == 0)
        {
            return Direction.WEST;
        }
        else if(xDiff == -1 && yDiff == -1)
        {
            return Direction.NORTHWEST;
        }

        System.err.println("Invalid path. Could not determine direction");
        return null;
    }
}
