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

	/**
	 * An abstraction for the map locations
	 * We implement comparable and override hashCode and equals so that we
	 * can properly use a PriorityQueue and HashMap with MapLocation
	 */
    class MapLocation implements Comparable<MapLocation>
    {
    	//The x and y coordinates of the location
        public int x, y;
        //The location we came from (null at head)
        public MapLocation cameFrom;
        //The f score of this location (f = g + h)
        public int f_score;
        //The g score of this location (g = cameFrom.g + 1);
        public int g_score;
        
        /**
         * Constructor when we only know or only need the x and y coordinates
         * @param x		the x coordinate
         * @param y		the y coorinate
         */
        public MapLocation(int x, int y){
        	this.x = x;
        	this.y = y;
        }
        
        /**
         * This is the default constructor the assignment came with
         * None of the code we added really uses it, but it was necessary to keep it
         * for the code the assignment came with
         * @param x the x coordinate
         * @param y the y coordinate
         * @param cameFrom	the MapLocation this once came from
         * @param cost	the cost to this location
         */
        public MapLocation(int x, int y, MapLocation cameFrom, float cost)
        {
            this(x,y);
            this.cameFrom = cameFrom;
        }
        
        /**
         * A constructor with set values for all the private variables
         * @param x the x coordinate
         * @param y the y coordinate
         * @param cameFrom	the MapLocation this once came from
         * @param cost	the cost to this location
         * @param f the f value (f = g + h)
         * @param g the g values
         */
        public MapLocation(int x, int y, MapLocation cameFrom, float cost, int f, int g)
        {
            this(x, y, cameFrom, cost);
            f_score = f;
            g_score = g;   
        }
        
        //Prints the x and y coordinate of this MapLocation in (x,y) format
        @Override
        public String toString()
        {
        	return "("+x+","+y+")";
        }
        
        //compareTo function to enable PriorityQueue
        //Simply subtract the other f score for this f score
		@Override
		public int compareTo(MapLocation o) {
			return (this.f_score - o.f_score);
		}
		
		//We needed to add a consistent hashCode for HashMaps to work
		//This is only based on the x and y coordinates
		//We use the default Integer hashCode, with a constant multiplier on the y
		@Override
		public int hashCode(){
			return Integer.valueOf(x).hashCode() + Integer.valueOf(x).hashCode()*23;
		}
		
		//We needed to add a consistent equals function for PriorityQueue and HashMaps to work
		//This is only based on the x and y coordinates because 2 MapLocations are equal
		//if they have the same coordinates, even if the other information is inconsistent.
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
    
    //For deciding whether or not we want to replan our path, one thing we
    //take into account is the dangerLevel.
    //For our purposes, dangerLevel is proportional to our concern that our
    //path is not optimal.
    //When dangerLevel >= MAX_DANGER_LEVEL, we will replan the path
    private int dangerLevel = 0;
    private final int MAX_DANGER_LEVEL = 4;
    
    //For deciding whether or not we want to replan our path, we also 
    //check to see if the enemy fotman is on our current path in within
    //MAX_LOOKAHEAD steps
    private final int MAX_LOOKAHEAD = 3;
    
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
     * This method should return true when the path needs to be replanned
     * and false otherwise. This will be necessary on the dynamic map where the
     * footman will move to block your unit.
     * 
     * There are two cases when we should replan:
     * 1) The dangerLevel is >= MAX_DANGER_LEVEL.  dangerLevel is related to
     * 		the 'danger' that we may be following a non-optimal path.
     * 		dangerLevel is incremented every time we think we are not on an
     * 		optimal pathm and then we replan when the dangerLevel hits the max.
     * 		This ensures that, when we replan the path because the enemy footman is in
     * 		our way, we check again soon to see if the footman moved out of the way.
     * 2) The enemy footman is within MAX_LOOKAHEAD steps ahead in out current chosen path.
     * 		If the enemy is in our way, we can't travel this path so we must replan.
     * 		However, we set dangerLevel to 1, which increments, ensuring that we will
     * 		try replanning again in MAX_DANGER_LEVEL - 1 steps.
     *
     * @param state		The state
     * @param history	The history
     * @param currentPath	The currentPath stack
     * @return	true if we should replan the path
     */
    private boolean shouldReplanPath(State.StateView state, History.HistoryView history, Stack<MapLocation> currentPath)
    {
        //Case 1: Check if danger level is too high
    	
    	//If we are in danger (danger Level == 0 when we are not in danger)
        if(dangerLevel != 0)
        {
        	//check if we are at max danger
	    	if(dangerLevel >= MAX_DANGER_LEVEL)
	        {
	        	//Set dangerLevel to 0 (we are no longer in danger, until we sense the
	        	//enemy footman again)
	        	dangerLevel = 0;
	        	//We should replan the path
	        	return true;
	        }
	    	//If danger is not max but also not zero, we must increment the danger level 
	    	//to ensure we'll try replanning the path soon
	    	dangerLevel++;
        }
        
        
        //Case 2: Check if the enemy footman is in the way of our current path
        //			(up to MAX_LOOKAHEAD future steps)
    	
        //Get the positions of our footman and the enemy footman
    	Unit.UnitView footmanUnit = state.getUnit(footmanID);
    	Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
    	
    	//We just use the x and y coordinates since there isn't a built in
    	//getMapLocationFunction()
        int footmanX = footmanUnit.getXPosition();
        int footmanY = footmanUnit.getYPosition();

        int enemyFootmanX = enemyFootmanUnit.getXPosition();
        int enemyFootmanY = enemyFootmanUnit.getYPosition();
        
        //Calculate the Chebyshev distance from our footman to the enemy footman
        int chebyshevDist = (int)Math.max( Math.abs((double)(footmanX - enemyFootmanX)),
										Math.abs((double)(footmanY - enemyFootmanY))) ;
        
        //We know Case 2 is false if the Chebyshev distance from our footman to the enemy
        //is > MAX_LOOKAHEAD, so no use in looking ahead in that case.
        if(chebyshevDist <= MAX_LOOKAHEAD){
        	//Now we check the next MAX_LOOKAHEAD steps in our path to see if the enemy
        	//footman is there
        
	        int stepsLookingAhead = 0;
	        //Loop through the first checkDist locations, or until the path is over
	        for(int i = currentPath.size()-1; 0 <= i && stepsLookingAhead < MAX_LOOKAHEAD; i--, stepsLookingAhead++)
	        {
	        	//Get the ith location
	        	MapLocation m = currentPath.get(i);
	        	
	        	//If the footman is spotted at this location, we want to replan the path
	        	if(m.x == enemyFootmanX && m.y == enemyFootmanY)
	        	{
	        		//Set dangerLevel to 1, ensuring that we will replan again in
	        		//at most MAX_DANGER_LEVEL - 1 steps
	        		dangerLevel = 1;
	        		//We want to replan the path because the footman is in our way
	        		return true;
	        	}
	        }
        }
        
        //If the danger level is okay, and the footman is not currently in our way
        //we do not replan
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
    	//Closed set is a HashMap in order to have constant .contains() check
    	Set<MapLocation> closedSet = new HashSet<MapLocation>();
    	
    	//Open set is a priority queue in order to have logn getMinF value
    		//A possible optimization would be to have an additional HashSet version of openSet
    		//because it would have a O(1) .contains() method rather than O(logn)
    		//The expense of this optimization would be that it requires double the space
    		// -- an expense which was not optimal for our smaller sample sizes
    	PriorityQueue<MapLocation> openSet = new PriorityQueue<MapLocation>();
    	
    	//The given MapLocation is not the right format for a HashSet and Priority Queue, 
    	//so we convert it and add it to the open set
    	start = new MapLocation(start.x, start.y, start.cameFrom, 0, hfun(start, goal), 0);
    	openSet.add(start);
    	
    	MapLocation current;
    	//While the open set is not empty
    	while(!openSet.isEmpty())
    	{
    		//Remove the best (lowest F value) location from the open set
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
					//Add g and f scores to neighbor
					int g_score_estimate = current.g_score + 1;
					neighbor = new MapLocation(neighbor.x, neighbor.y, current, 0, 
							g_score_estimate + hfun(neighbor, goal), g_score_estimate);
					
					//Add it to the open set
					openSet.add(neighbor);
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