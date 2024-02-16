// --== CS400 File Header Information ==--
// Name: Phoenix Bach Pham
// Email: pbpham@wisc.edu
// Group and Team: G34
// Group TA: Zheyang Xiong
// Lecturer: Florian Heimrl
// Notes to Grader: <optional extra notes>
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.params.ParameterizedTest;

import java.util.*;
import java.util.NoSuchElementException;

/**
 * This class extends the BaseGraph data structure with additional methods for
 * computing the total cost and list of node data along the shortest path
 * connecting a provided starting to ending nodes. This class makes use of
 * Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number>
        extends BaseGraph<NodeType, EdgeType>
        implements GraphADT<NodeType, EdgeType> {

    /**
     * While searching for the shortest path between two nodes, a SearchNode
     * contains data about one specific path between the start node and another
     * node in the graph. The final node in this path is stored in its node
     * field. The total cost of this path is stored in its cost field. And the
     * predecessor SearchNode within this path is referened by the predecessor
     * field (this field is null within the SearchNode containing the starting
     * node in its node field).
     * <p>
     * SearchNodes are Comparable and are sorted by cost so that the lowest cost
     * SearchNode has the highest priority within a java.util.PriorityQueue.
     */
    protected class SearchNode implements Comparable<SearchNode> {
        public Node node;
        public double cost;
        public SearchNode predecessor;

        public SearchNode(Node node, double cost, SearchNode predecessor) {
            this.node = node;
            this.cost = cost;
            this.predecessor = predecessor;
        }

        public int compareTo(SearchNode other) {
            if (cost > other.cost)
                return +1;
            if (cost < other.cost)
                return -1;
            return 0;
        }
    }

    /**
     * Constructor that sets the map that the graph uses.
     *
     * @param map the map that the graph uses to map a data object to the node
     *            object it is stored in
     */
    public DijkstraGraph(MapADT<NodeType, Node> map) {
        super(map);
    }

    /**
     * This helper method creates a network of SearchNodes while computing the
     * shortest path between the provided start and end locations. The
     * SearchNode that is returned by this method is represents the end of the
     * shortest path that is found: it's cost is the cost of that shortest path,
     * and the nodes linked together through predecessor references represent
     * all of the nodes along that shortest path (ordered from end to start).
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return SearchNode for the final end node within the shortest path
     * @throws NoSuchElementException when no path from start to end is found
     *                                or when either start or end data do not
     *                                correspond to a graph node
     */
    protected SearchNode computeShortestPath(NodeType start, NodeType end) {

        // implement in step 5.3

        PlaceholderMap<NodeType, Node> visitedNodes = new PlaceholderMap<>();
        PriorityQueue<SearchNode> pq = new PriorityQueue<>();
        SearchNode currSN = new SearchNode(this.nodes.get(start), 0, null);
        pq.add(currSN);

        while (!pq.isEmpty()) {

            currSN = pq.remove();
            if (currSN.node.equals(this.nodes.get(end))) {
                return currSN;
            }
            if (visitedNodes.containsKey(currSN.node.data)) {
                continue;
            }
            visitedNodes.put(currSN.node.data, currSN.node);
            Iterator currEdgeIterator = currSN.node.edgesLeaving.iterator();

            while (currEdgeIterator.hasNext()) {
                Edge currEdge = (Edge)currEdgeIterator.next();
                Double weight = currSN.cost + currEdge.data.doubleValue();
                SearchNode succ = new SearchNode(currEdge.successor, weight, currSN);
                pq.add(succ);
            }
        }
        throw new NoSuchElementException();
    }

    /**
     * Returns the list of data values from nodes along the shortest path
     * from the node with the provided start value through the node with the
     * provided end value. This list of data values starts with the start
     * value, ends with the end value, and contains intermediary values in the
     * order they are encountered while traversing this shorteset path. This
     * method uses Dijkstra's shortest path algorithm to find this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return list of data item from node along this shortest path
     */
    public List<NodeType> shortestPathData(NodeType start, NodeType end) {
        SearchNode currNode = this.computeShortestPath(start, end);
        List<NodeType> shortestPath = new LinkedList<>();

        while(currNode != null) {
            shortestPath.add(0, currNode.node.data);
            currNode = currNode.predecessor;
        }
        return shortestPath;
    }

    /**
     * Returns the cost of the path (sum over edge weights) of the shortest
     * path freom the node containing the start data to the node containing the
     * end data. This method uses Dijkstra's shortest path algorithm to find
     * this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return the cost of the shortest path between these nodes
     */
    public double shortestPathCost(NodeType start, NodeType end) {
        // implement in step 5.4
        SearchNode shortestPath = this.computeShortestPath(start, end);
        return shortestPath.cost;
    }

    // TODO: implement 3+ tests in step 4.1

}
