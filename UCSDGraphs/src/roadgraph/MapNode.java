package roadgraph;
import geography.GeographicPoint;
import java.util.Comparator;


/**
 * @author Zach Dischner
 *
 * Simple class to represent a map Node (GeographicPoint) abstractor
 *  -- Class can be used for:
 *      * Unweighted graph searching (BFS/DFS) by ignoring `distFrom*` args when forming
 *      * Weighted graph searching from (Dijkstra) by ignoring `distFromDest` arg when forming
 *      * Directed weighted graph searching (A*) by providing known `distFromSource` and estimated `DistFromDest`
 *       args when forming.
 *    In the latter two cases, the score() method can be used to prioritize this Node for searching agains other nodes.
 *    See `MapGraph.java` for example usage
 *
 */
public class MapNode implements Comparable<MapNode> {

    /**
     * Constructor
     */
    MapGraph.VisitState visitState;
    GeographicPoint point;
    double distFromSource;   // Intention: True path distance to the source
    double distFromDest;     // Intention: Estimated distance to destination (A* search metric)

    public MapNode(GeographicPoint point) {
        this(point, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    public MapNode(GeographicPoint point, double distFromSource, double distFromDest){
        this.point = point;
        this.visitState = MapGraph.VisitState.UNVISITED;
        this.distFromSource = distFromSource;
        this.distFromDest = distFromDest;
    }

//    @Override
    // http://www.digizol.com/2008/07/java-sorting-comparator-vs-comparable.html
    public int compareTo(MapNode b){
        if (this.antiScore() < b.antiScore()) {return -1;}
        if (this.antiScore() > b.antiScore()) {return 1;}
        return 0;
    }

    // Get the 'antiScore' of this node - Scores are different depending on if this is used for dijkstra or bfs or A*
    // Alternate use for scoring instead of the `compareTo` interface which is just more complicated.
    // 'Antiscore' because the higher this value, the worse this node is
    public double antiScore(){
//        System.out.format("\t\t>Node (%3.5f, %3.5f) score: %3.5f",this.point.getX(), this.point.getY(), this.distFromSource);
        return this.distFromSource + this.distFromDest;
    }

    public String toString() {
        return "Node: [" + this.point.toString() + "]("+this.distFromSource+")";
    }
}