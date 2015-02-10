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
        public int f_score;
        public int g_score;
        
        public MapLocation(int x, int y){
        	this.x = x;
        	this.y = y;
        }

        public MapLocation(int x, int y, MapLocation cameFrom, float cost)
        {
            this(x,y);
            this.cameFrom = cameFrom;
        }
        
        public MapLocation(int x, int y, MapLocation cameFrom, float cost, int f, int g)
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
			return (this.f_score - o.f_score);
		}
		
		@Override
		public int hashCode(){
			return Integer.valueOf(x).hashCode() + Integer.valueOf(x).hashCode()*23;
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

    public AstarAgent(int playernum)
    {
        super(playernum);
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
        
        return middleStep(newstate, statehistory);
    }

    @Override
    public Map<Integer, Action> middleStep(State.StateView newstate, History.HistoryView statehistory) {
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
    	//If there is no enemyFootman, there should never be a reason to replan the path
    	if(state.getUnit(enemyFootmanID) == null) return false;
    	
    	//Otherwise the positions of our footman and the enemy footman
    	Unit.UnitView footmanUnit = state.getUnit(footmanID);
    	Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
    	
    	//We just use the x and y coordinates since there isn't a built in
    	//getMapLocationFunction()
        int footmanX = footmanUnit.getXPosition();
        int footmanY = footmanUnit.getYPosition();

        int enemyFootmanX = enemyFootmanUnit.getXPosition();
        int enemyFootmanY = enemyFootmanUnit.getYPosition();
        
        
        if(Math.abs(footmanX-enemyFootmanX)+Math.abs(footmanY-enemyFootmanY)>2)
        	return false;
        
        //Check if the footman is within checkDist steps ahead of us on our path
        int checkDist = 4;
        Vector<MapLocation> v = currentPath;
        int count = 0;
        //Loop through the first checkDist locations ahead in our proposed path
        for(int i = v.size()-1; i > 0; i--)
        {
        	//Get the ith location
        	MapLocation m = v.get(i);
        	
        	//If the footman is at this location, we want to replan the path
        	if(m.x == enemyFootmanX && m.y == enemyFootmanY)
        	{
        		return true;
        	}
        	//After checking checkDist steps, don't check any further
        	//Because the enemy footman might move as we get closer
        	if(count == checkDist ) break;
        	count++;
        }
        
        //If the footman isn't too close/on our path, no reason to replan the path
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
    	//Closed set is a HashMap to have constant .contains() check
    	Set<MapLocation> closedSet = new HashSet<MapLocation>();
    	//Open set is a priority queue to have logn getMax F value
    	//A possible optimization would be to also have a HashSet openset, so
    	//it would have a O(1) .contains() method rather than O(logn)
    	PriorityQueue<MapLocation> openSet = new PriorityQueue<MapLocation>();
    	
    	//The given MapLocation is not the right format for a HashSet and Priority Queue, 
    	//so we convert it
    	MapLocation s = new MapLocation(start.x, start.y, start.cameFrom, 0, hfun(start, goal), 0);
    	openSet.add(s);
    	
    	MapLocation current;
    	//While the open set is not empty
    	while(!openSet.isEmpty())
    	{
    		//Remove the best (highest F value) location from the open set
    		//remove() for a priority queue will remove the best location
    		//based on the .compareTo() function we have defined
    		current = openSet.remove();
    		
    		//If this is the goal, we've completed the search
    		if(current.equals(goal))
    		{
    			System.out.println("done");
    			return reconstructPath(start,current);
    		}
    		//Otherwise, add this location to the closed set
    		closedSet.add(current);
    		//Get the successors of this node
    		Set<MapLocation> sucessors = getSucessors(current, enemyFootmanLoc, resourceLocations, xExtent, yExtent);
    		
    		//Check each successor
    		for(MapLocation neighbor : sucessors)
    		{
    			//If we have explored it already, skip it
    			if(closedSet.contains(neighbor))
    				continue;
    			
    			//If it is not in the open set
				if(!openSet.contains(neighbor)){
					//Add it to the open set
					// -- But first we have to calculate it's g and f values because
					// -- get successors does not do this (since some successor are skipped
					// -- because they are in the open set, it is better to not make this
					// -- calculation for every successor in getSuccessors()
					//The g score is just 1 plus the g score of the current node
					int g_score_estimate = current.g_score + 1;
					MapLocation n = new MapLocation(neighbor.x, neighbor.y, current, 0, 
							g_score_estimate + hfun(neighbor, goal), g_score_estimate);
					openSet.add(n);
					
				}
    			
    		}
    	}
        // there is no path
    	System.out.println("No Available Path");
        return new Stack<MapLocation>();
    }
 
    /**
     * Rebuilds the path by taking the parent location and backtracking through its cameFrom
     * @param start The start of the path	(where the footman starts)
     * @param end	The end of the path (the fort)
     * @return	a stack form of the path
     */
    private Stack<MapLocation> reconstructPath(MapLocation start, MapLocation end)
    {
    	//Initialize our returned stack
    	Stack<MapLocation> toReturn = new Stack<MapLocation>();
    	
    	//We start from the end
    	//We do not include the end and start in the stack because the end is the
    	//fort (we cannot travel there) and the start is our footman.
    	MapLocation current = end;
    	//Until we reach the start, keep pulling from the cameFrom value
    	while(!current.equals(start))
    	{
    		current = current.cameFrom;
    		//Push the cameFrom to our return stack
    		toReturn.push(current);
    	}
    	
    	//Return the completed path
    	return toReturn;
    	
    }
    
    /**
     * Finds all the possible children locations from the current location
     * @param current The current location
     * @param enemyFootmanLoc	The Location of the enemy footman
     * @param resourceLocations	The set of resource locations
     * @param xExtent	The extent of the x coordinate of the grid
     * @param yExtent	The extent of the y coordinate of the grid
     * @return	Returns a set of the successors
     */
    private Set<MapLocation> getSucessors(MapLocation current, MapLocation enemyFootmanLoc, Set<MapLocation> resourceLocations, int xExtent, int yExtent)
    {
    	//Initialize the returned set
    	Set<MapLocation> toReturn = new HashSet<MapLocation>();
    	//We must check all the grid locations around current
    	//-- up, down, left, right, and diagonoals
    	for(int i = -1; i<=1; i++)
    	{
    		for(int j = -1; j<=1; j++)
    		{
    			//Make the map location of this potential succesor
    			MapLocation loc = new MapLocation(current.x+i,current.y+j);
    			//Check if there's anything already in this potential successor
    			if(isValidLocation(current, loc, enemyFootmanLoc, resourceLocations, xExtent,  yExtent))
    			{
    				//If there's nothing at loc, it is a successor, so add it to the return set
    				toReturn.add(loc);
    			}
    		}
    	}
    	return toReturn;
    	
    }
    
    /*
     * checks if the location is a location we can move to
     * not updated for the dynamic case yet
     */
    /**
     * Checks if a location is one that is possible to move to, ie it is empty.
     * @param current The current location
     * @param loc	The location we are checking
     * @param enemyFootmanLoc	The enemy footman location
     * @param resourceLocations	The set of resource locations
     * @param xExtent	The extent of the x coordinate of the grid
     * @param yExtent	The extent of the y coordinate of the grid
     * @return	True if the loc is empty, false otherwise
     */
    private boolean isValidLocation(MapLocation current, MapLocation loc, MapLocation enemyFootmanLoc, Set<MapLocation> resourceLocations, int xExtent, int yExtent)
    {
    	//If there is a resource at loc, return false
    	if (resourceLocations.contains(loc))
    		return false;
    	//If the location is the current (our fighter is currently there) return false
    	else if(loc.equals(current))
    		return false;
      	//If the location is off the board, return false
    	else if(loc.x<0 || loc.y<0 || loc.x>=xExtent || loc.y>=yExtent)
    		return false;
    	//If there is an enemy footman, and it is at loc, return false
    	else if(enemyFootmanLoc!= null && loc.equals(enemyFootmanLoc))
    		return false;
    	//Otherwise, loc is valid
    	else
    		return true;
    }
    
    /**
     * The heuristic described by the assignment (Chebyshev distance).
     * @param start The start location (location we are testing from)
     * @param goal	The goal location
     * @return	the chebyshev distance (MAX( |start.x = goal.x|, |start.y - goal.y| )
     */
    private int hfun(MapLocation start, MapLocation goal)
    {
    	return Math.max((Math.abs(start.x - goal.x)),(Math.abs(start.y - goal.y)));
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
