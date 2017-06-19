package basicgraph;

import java.util.*;

/** A class that implements a directed graph. 
 * The graph may have self-loops, parallel edges. 
 * Vertices are labeled by integers 0 .. n-1
 * and may also have String labels.
 * The edges of the graph are not labeled.
 * Representation of edges via adjacency lists.
 * 
 * @author UCSD MOOC development team and YOU
 *
 */
public class GraphAdjList extends Graph {


	private Map<Integer,ArrayList<Integer>> adjListsMap;
	
	/** 
	 * Create a new empty Graph
	 */
	public GraphAdjList () {
		adjListsMap = new HashMap<Integer,ArrayList<Integer>>();
	}

	/** 
	 * Implement the abstract method for adding a vertex. 
	 */
	public void implementAddVertex() {
		int v = getNumVertices();
		ArrayList<Integer> neighbors = new ArrayList<Integer>();
		adjListsMap.put(v,  neighbors);
	}
	
	/** 
	 * Implement the abstract method for adding an edge.
	 * @param v the index of the start point for the edge.
	 * @param w the index of the end point for the edge.  
	 */
	public void implementAddEdge(int v, int w) {
		(adjListsMap.get(v)).add(w);

	}
	
	/** 
	 * Implement the abstract method for finding all 
	 * out-neighbors of a vertex.
	 * If there are multiple edges between the vertex
	 * and one of its out-neighbors, this neighbor
	 * appears once in the list for each of these edges.
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */	
	public List<Integer> getNeighbors(int v) {
		return new ArrayList<Integer>(adjListsMap.get(v));
	}

	/** 
	 * Implement the abstract method for finding all 
	 * in-neighbors of a vertex.
	 * If there are multiple edges from another vertex
	 * to this one, the neighbor
	 * appears once in the list for each of these edges.
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */	
	public List<Integer> getInNeighbors(int v) {
		List<Integer> inNeighbors = new ArrayList<Integer>();
		for (int u : adjListsMap.keySet()) {
			//iterate through all edges in u's adjacency list and 
			//add u to the inNeighbor list of v whenever an edge
			//with startpoint u has endpoint v.
			for (int w : adjListsMap.get(u)) {
				if (v == w) {
					inNeighbors.add(u);
				}
			}
		}
		return inNeighbors;
	}
	 

	/** 
	 * Implement the abstract method for finding all 
	 * vertices reachable by two hops from v.
	 *
	 * 2 methods: 1. Add to a list of 2 hop neighbors, checking to see if it exists in the list first
	 * 			  2. Create a Set (HashSet) of 2 hop neighbors, then extract keys into a new list
	 * 
	 * @param v the index of vertex.
	 * @return List<Integer> a list of indices of vertices.  
	 */		
	 public List<Integer> getDistance2(int v) {
//		 Set neighborSet = new HashSet();
		 List<Integer> neighbors = new ArrayList<>();
		 for (int neighbor: getNeighbors(v)){
		 	for (int subneighbor: getNeighbors(neighbor)){
			 	//Method 1 - put in a new HashSet() and return the keys
//		 		neighborSet.add(subneighbor);
		 		// Method2 - Put in list if it is not there already   (Or duplicate additions for non-uniqueness)
//		 		if (!neighbors.contains(subneighbor)){neighbors.add(subneighbor);}
				neighbors.add(subneighbor);
			}
		 }
//		 neighbors.addAll(neighborSet);
		 return neighbors;
	}
	
	/**
	 * Generate string representation of adjacency list
	 * @return the String
	 */
	public String adjacencyString() {
		String s = "Adjacency list";
		s += " (size " + getNumVertices() + "+" + getNumEdges() + " integers):";

		for (int v : adjListsMap.keySet()) {
			s += "\n\t"+v+": ";
			for (int w : adjListsMap.get(v)) {
				s += w+", ";
			}
		}
		return s;
	}




}
