/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;
import java.util.List;
import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 *
 */
public class MapGraph {
	// Threshold for when two coordinates are *the same*
	public double dist_threshold = 0.001;
	// Map Graph will be stored as an *Adjacency List*, because the nodes will likely be sparsely connected
	public Map<GeographicPoint, List<MapEdge>> edgeMap; // edgeMap maps a point to a list of all outgoing MapEdges
	public Map<GeographicPoint, MapNode> nodeMap;  // Not crazy useful since all the Node has is visit state.
												      		// May just be better to define a __repr__ like MapNode key
															// Keeping this way bc interface calls for geo points etc

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph()
	{
		// Create graph definition variables
		this.nodeMap = new HashMap<>() ;
		this.edgeMap = new HashMap<>() ;
	}


	/**
	 * Resets all nodes to the UNVISITED state for re-searching
	 */
	public void reset()
	{
		// Mark each node as unvisited  **MIGHT BE BETTER OFF JUST HAVING THIS BE DONE BY DEFAULT IN SEARCHES**
		this.nodeMap.forEach((k, v) -> v.visitState = VisitState.UNVISITED);
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		// Number of vertices is simple.
		return this.nodeMap.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		// Vertices are the Keys in the nodeMap object
		return this.nodeMap.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
//		int edgeCount = 0;
//		for (Map.Entry<GeographicPoint, List<MapEdge>> entry : this.edgeMap.entrySet()) {
//			edgeCount += entry.getValue().size();
//		}
//		return edgeCount;
		final int[] edgeCount = {0};
		this.edgeMap.forEach((k, v) -> edgeCount[0] += v.size());
		return edgeCount[0];
	}



	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// Failure conditions
//		System.out.format("Number of nodes: %d\n", this.nodeMap.size());
		if (location==null || nodeMap.containsKey(location)) {
			System.out.println("Node already exists! " + location.toString());
			return false;
		}

		// Add vertex to `nodeMap`, container mapping Geo coords to a MapNode object
		this.nodeMap.put(location, new MapNode(location));

		// Also initialize the vertexMap mapping
		this.edgeMap.put(location, new ArrayList<>());  // point has no outgoing edges at this point

		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3

		// Validity checks: Are the points valid and the points exist in the graph?
		if (from==null) {throw new IllegalArgumentException("`from` point cannot be null!");}
		if (to==null) {throw new IllegalArgumentException("`to` point cannot be null!");}
		if (!this.nodeMap.containsKey(from)) {throw new IllegalArgumentException("`from` point  " + from.toString() + " is not in the graph!");}
		if (!this.nodeMap.containsKey(to)) {throw new IllegalArgumentException("`from` point  " + to.toString() + " is not in the graph!");}

		// Validity checks: Do edge properties make sense?
		if (roadName==null) {throw new IllegalArgumentException("`roadName` cannot be null!");}
		if (roadType==null) {throw new IllegalArgumentException("`roadType` cannot be null!");}
		if (length<0) {throw new IllegalArgumentException("`length` cannot be less than 0!");}

		// All checks passed! Add to the `edgeMap` map
		this.edgeMap.get(from).add( new MapEdge(from, to, roadName, roadType, length));

	}


	/** Trace lineage from dest back to source through a parental mapping. Terminates when a parent is null
	 *
	 *
	 */
	public List<GeographicPoint> parentTraceback(GeographicPoint leafNode, Map<GeographicPoint,GeographicPoint> parentMap){
		// Traceback and return path from `start` to `goal` by examining each object's parents
		List<GeographicPoint> path = new ArrayList<>();  // Fill in: [`goal` -> `goal.parent`... `start`] and reverse
		while (leafNode != null){   // Not especially pretty...
			path.add(leafNode);
			leafNode = parentMap.get(leafNode);
		}
		Collections.reverse(path);
		return path;
	}


	/** Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}

	/** Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// First off, set each node to UNVISITED for this new search
		this.reset();

		// Initialize Search Queue and Parent Mapping
		Queue<GeographicPoint> queue = new LinkedList<>();
		queue.add(start);
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<>();
		parentMap.put(start,null);
		boolean found = false;
		GeographicPoint thisNode = null;


		// Start the search!
		while (!queue.isEmpty()) {
			// Deque, add the previously explored to the parent mapping
			thisNode = queue.remove();
			System.out.println("Exploring node: " + thisNode.toString());// + " Queue length: " + queue.size());
			nodeSearched.accept(thisNode);
			nodeMap.get(thisNode).visitState = VisitState.VISITED;  // Mark node as visited. Avoids loops and whatnot

			// Reached our goal?
//			System.out.println("\t\t\tThis (" + thisNode.toString() + " distance to goal " + goal.toString() + "?: " + thisNode.distance(goal));
			if (thisNode.distance(goal) < dist_threshold) {
//				System.out.println("Found!");
				found = true;
				break;
			}

			// Queue up more unexplored nodes for exploration. AKA each endpoint in this node's outgoing edges
			for (MapEdge edge: this.edgeMap.get(thisNode)){
				if (this.nodeMap.get(edge.end).visitState == VisitState.UNVISITED){
					this.nodeMap.get(edge.end).visitState = VisitState.VISITING;
//					System.out.println("\tAdding child node: " + edge.end.toString());
					queue.add(edge.end);
					parentMap.put(edge.end, thisNode);
				}
			}
		}

		// Goal was not reached! Return empty list
		if (found==false){			System.out.println("\t\tNever found the goal :-(");
			return new ArrayList<>();}

		List<GeographicPoint> path = parentTraceback(thisNode, parentMap);
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start,
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Different than BFS, just seeing which I like better. Use a visited mapping instead of changing class var
		Set<GeographicPoint> visited = new HashSet<>();

		// Initialize Search Queue, Parent Mapping, and distance mapping (3 options. Nice ;-))
//		Queue<MapNode> queue = new PriorityQueue<>(this.edgeMap.size(),(a,b) -> a.distFromSource < b.distFromSource ? -1 :1);
//		Queue<MapNode> queue = new PriorityQueue<>(this.edgeMap.size(),(a,b) -> a.antiScore() < b.antiScore() ? -1 :1);
		Queue<MapNode> queue = new PriorityQueue<>(this.edgeMap.size()); // MapNode has built in compare() functionality
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<>();
		parentMap.put(start,null);

		Map<GeographicPoint,Double> distanceMap = new HashMap<>();
		this.nodeMap.forEach((k,v)->distanceMap.put(k, Double.MAX_VALUE)); // Initialize with infinity for all distances

		//
		queue.add(new MapNode(start,0, 0));
		boolean found = false;
		MapNode thisNode = null;
		double newDistFromSource;

		while (!queue.isEmpty()){
//			System.out.println("\nQueue is: ");
//			for (MapNode node : queue){
//				System.out.format("\t-Node: %s, dist: %3.3f\n", node.toString(), node.distFromSource);
//			}
//			System.out.println();
			thisNode = queue.remove(); // Get highest priority (lowest distance node) from queue
//			System.out.println("Evaluating node: " + thisNode.toString());



			if (visited.contains(thisNode.point)){continue;}
			visited.add(thisNode.point);
//			System.out.format("\tVisiting node " + thisNode.toString() + " with path length %4.5f", thisNode.distFromSource);

			//Otherwise, explore all paths from here			visited.add(thisNode.point);
//			System.out.format("\tputting " + thisNode.point.toString() + " with dist (" + thisNode.distFromSource + ")");
			distanceMap.put(thisNode.point, thisNode.distFromSource);
			if (thisNode.point.distance(goal) < dist_threshold){
				found=true;
				System.out.format("\nDijkstra Found goal in %d visits, Path Len: %3.5f\n",visited.size(), thisNode.distFromSource);
				break;
			}			nodeSearched.accept(thisNode.point);

			// Queue up more unexplored nodes for exploration. AKA each endpoint in this node's outgoing edges
			for (MapEdge edge: this.edgeMap.get(thisNode.point)){
				if (!visited.contains(this.nodeMap.get(edge.end))){
					newDistFromSource = thisNode.distFromSource + edge.length; // Total path to _here_ plus length to next node
					if ( newDistFromSource < distanceMap.get(edge.end)){
						distanceMap.put(edge.end, newDistFromSource);
						queue.add( new MapNode(edge.end, newDistFromSource, 0));
						parentMap.put(edge.end, thisNode.point);
//						System.out.println("\t" + edge.end.toString() + " parent now is -->  " +
//								thisNode.point.toString() + " with len " + newDistFromSource);
//						System.out.format("Now: " + edge.end.toString() + "Has distacne: " + distanceMap.get(edge.end).toString());
					}
				}
			}

		}

		// Goal was not reached! Return empty list
		if (!found){System.out.println("\t\tNever found the goal :-(");
			return new ArrayList<>();}

		List<GeographicPoint> path = parentTraceback(thisNode.point, parentMap);
//		System.out.println("Dijkstra Searched " + visited.size() + " nodes");
		return path;
	}

	/** Find the path from start to goal using A-Star search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}

	/** Find the path from start to goal using A-Star search
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start,
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Different than BFS, just seeing which I like better. Use a visited mapping instead of changing class var
		Set<GeographicPoint> visited = new HashSet<>();

		// Initialize Search Queue, Parent Mapping, and distance mapping (3 options. Nice ;-))
//		Queue<MapNode> queue = new PriorityQueue<>(this.edgeMap.size(),(a,b) -> a.distFromSource < b.distFromSource ? -1 :1);
//		Queue<MapNode> queue = new PriorityQueue<>(this.edgeMap.size(),(a,b) -> a.antiScore() < b.antiScore() ? -1 :1);
		Queue<MapNode> queue = new PriorityQueue<>(this.edgeMap.size()); // MapNode has built in compare() functionality
		Map<GeographicPoint,GeographicPoint> parentMap = new HashMap<>();
		parentMap.put(start,null);

		Map<GeographicPoint,Double> distanceMap = new HashMap<>();
		this.nodeMap.forEach((k,v)->distanceMap.put(k, Double.MAX_VALUE)); // Initialize with infinity for all distances

		//
		queue.add(new MapNode(start,0, 0));
		boolean found = false;
		MapNode thisNode = null;
		double newDistFromSource;
		double estDistFromDest;

		while (!queue.isEmpty()){
			thisNode = queue.remove(); // Get highest priority (lowest distance node) from queue
//			System.out.println("Evaluating node: " + thisNode.toString());



			if (visited.contains(thisNode.point)){continue;}
			visited.add(thisNode.point);
//			System.out.format("\tVisiting node " + thisNode.toString() + " with path length %4.5f", thisNode.distFromSource);

			//Otherwise, explore all paths from here
			distanceMap.put(thisNode.point, thisNode.distFromSource);
			if (thisNode.point.distance(goal) < dist_threshold){
				found=true;
				System.out.format("\nA* Found goal in %d visits, Path Len: %3.5f\n", visited.size(), thisNode.distFromSource);
				break;
			}
			nodeSearched.accept(thisNode.point);

			// Queue up more unexplored nodes for exploration. AKA each endpoint in this node's outgoing edges
			for (MapEdge edge: this.edgeMap.get(thisNode.point)){
				if (!visited.contains(this.nodeMap.get(edge.end))){
					newDistFromSource = thisNode.distFromSource + edge.length; // Total path to _here_ plus length to next node
					if ( newDistFromSource < distanceMap.get(edge.end)){
						distanceMap.put(edge.end, newDistFromSource);

						queue.add( new MapNode(edge.end, newDistFromSource, goal.distance(edge.end)));
						parentMap.put(edge.end, thisNode.point);
//						System.out.println("\t" + edge.end.toString() + " parent now is -->  " +
//								thisNode.point.toString() + " with len " + newDistFromSource);
//						System.out.format("Now: " + edge.end.toString() + "Has distacne: " + distanceMap.get(edge.end).toString());
					}
				}
			}

		}

		// Goal was not reached! Return empty list
		if (!found){System.out.println("\t\tNever found the goal :-(");
			return new ArrayList<>();}

		List<GeographicPoint> path = parentTraceback(thisNode.point, parentMap);
//		System.out.println("A* Searched " + visited.size() + " nodes");
		return path;
	}

	public void printMe(){
		System.out.format("\n\n----Geo Map----: Nodes: %d, Edges: %d\n", this.getNumVertices(), this.getNumEdges());
		for  (Map.Entry<GeographicPoint, List<MapEdge>> entry : this.edgeMap.entrySet()){
			System.out.format("Node: %s:\n", entry.getKey().toString());
			for (MapEdge edge: entry.getValue()){
				System.out.format("\t\t--> Endpoint (%s) via '%s', distance: %3.3f\n", edge.end.toString(), edge.roadName, edge.length);
			}
		}
		System.out.println("---------------------------------------------------");
	}

	// Helper classes (defined in this class? What a pain in the ass
	public enum VisitState {
		UNVISITED,
		VISITING,
		VISITED;
	}



	/**
	 * @author Zach Dischner
	 *
	 * Simple class to represent a map Node (GeographicPoint) abstractor
	 *
	 */
	public class MapEdge {

		/**
		 * Constructor
		 */
		GeographicPoint start;
		GeographicPoint end;
		String roadName;
		String roadType;
		double length;

		public MapEdge(GeographicPoint start, GeographicPoint end, String roadName, String roadType, double length) {
			this.start = start;
			this.end = end;
			this.roadName = roadName;
			this.roadType = roadType;
			this.length = length;
//			System.out.format("New edge! ("+start.toString()+"-->"+end.toString()+") ["+roadName+","+roadType+",%f]\n",length);
		}

		public String toString() {
			return "Edge: (" + this.start.toString() + "  --> " + this.end.toString() + " )";
		}
	}


	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		// You can use this method for testing.


		/* Here are some test cases you should try before you attempt
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		System.out.format("Loaded test map has %d nodes and %d edges", simpleTestMap.getNumVertices(),simpleTestMap.getNumEdges());
		simpleTestMap.printMe();

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("\n---------------------- Test 1 -----------------------------");
		System.out.println("\n*Test 1 using simpletest: BFS from " + testStart.toString() + " --> " + testEnd.toString());
		System.out.println("\tShould be: (1,1) --> (4,1) --> (7,3) --> (8,-1)");
		List<GeographicPoint> path = simpleTestMap.bfs(testStart, testEnd);
		System.out.format("\tLength of BFS path traversed: %d\n",path.size());
		for (GeographicPoint point: path){
			System.out.format("  ->  (%s)", point.toString());
		}
		System.out.println("\n--------------------------------------------------------------");


		System.out.println("\n-------------------------- Test 2 --------------------------");
		simpleTestMap.reset();
		GeographicPoint testStart2 = new GeographicPoint(4.0, -1.0);
		GeographicPoint testEnd2 = new GeographicPoint(6.5, 0.0);
		System.out.println("\n\n*Test 2 using simpletest: BFS from " + testStart2.toString() + " --> " + testEnd2.toString());
		System.out.println("\tShould be: (4,-1) --> (8,-1) --> (6.5,0)");
		List<GeographicPoint> path2 = simpleTestMap.bfs(testStart2, testEnd2);
		System.out.format("\tLength of BFS path traversed: %d\n",path2.size());
		for (GeographicPoint point: path2){
			System.out.format("  ->  (%s)", point.toString());
		}
		System.out.println("\n--------------------------------------------------------------");



		System.out.println("\n---------------------- Test 3 ----------------------------");

		System.out.format("\n\nTest 3 using simpletest: Dijkstra should be 9 and AStar should be 5\n\n");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.format("Dijkstra Route: ");
		for (GeographicPoint point: testroute){
			System.out.format("  ->  (%s)", point.toString());
		}

		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.format("\nA* Route:       ");
		for (GeographicPoint point: testroute2){
			System.out.format("  ->  (%s)", point.toString());
		}
		System.out.println("\n--------------------------------------------------------------");



		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("\n\n-----------------------Test 4---------------------------");
		System.out.println("Test 4 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println("\n--------------------------------------------------------------");



		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("\n\n-----------------------Test 5---------------------------");
		System.out.println("Test 5 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.format("\nDijkstra Route: ");
		for (GeographicPoint point: testroute){
			System.out.format("  ->  (%s)", point.toString());
		}

		System.out.format("\nA* Route:       ");
		for (GeographicPoint point: testroute2){
			System.out.format("  ->  (%s)", point.toString());
		}
		System.out.println("\n--------------------------------------------------------------");




		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);



	}

}
