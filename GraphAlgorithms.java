import java.util.*;

/**
 * Your implementations of various graph algorithms.
 *
 * @author Anthony De Santiago
 * @version 1.0
 */
public class GraphAlgorithms {

    /**
     * Perform breadth first search on the given graph, starting at the start
     * Vertex.  You will return a List of the vertices in the order that
     * you visited them.  Make sure to include the starting vertex at the
     * beginning of the list.
     *
     * When exploring a Vertex, make sure you explore in the order that the
     * adjacency list returns the neighbors to you.  Failure to do so may
     * cause you to lose points.
     *
     * The graph passed in may be directed or undirected, but never both.
     *
     * You may import/use {@code java.util.Queue}, {@code java.util.Set},
     * {@code java.util.Map}, {@code java.util.List}, and any classes
     * that implement the aforementioned interfaces.
     *
     * @throws IllegalArgumentException if any input is null, or if
     *         {@code start} doesn't exist in the graph
     * @param start the Vertex you are starting at
     * @param graph the Graph we are searching
     * @param <T> the data type representing the vertices in the graph.
     * @return a List of vertices in the order that you visited them
     */
    public static <T> List<Vertex<T>> breadthFirstSearch(Vertex<T> start,
                                                         Graph<T> graph) {
        //TODO BFS
        if (start == null || graph == null) {
            throw new IllegalArgumentException("Either start or graph was null!");
        }
        List<Vertex<T>> known = new ArrayList<>();
        BFS(graph, start, known);
        return known;
    }

    private static <T> void BFS(Graph<T> g, Vertex<T> s, List<Vertex<T>> known) {
        known.add(s);
        List<Vertex<T>> level = new ArrayList<>();
        level.add(s);
        while (!level.isEmpty()) {
            List<Vertex<T>> nextLevel = new ArrayList<>();
            for (Vertex<T> u : level) {
                for (Vertex<T> v : outGoingVertices(g, u)) {
                    if (!known.contains(v)) {
                        known.add(v);
                        nextLevel.add(v);
                    }
                }
                level = nextLevel;
            }
        }


    }

    /**
     * Perform depth first search on the given graph, starting at the start
     * Vertex.  You will return a List of the vertices in the order that
     * you visited them.  Make sure to include the starting vertex at the
     * beginning of the list.
     *
     * When exploring a Vertex, make sure you explore in the order that the
     * adjacency list returns the neighbors to you.  Failure to do so may
     * cause you to lose points.
     *
     * The graph passed in may be directed or undirected, but never both.
     *
     * You MUST implement this method recursively.
     * Do not use any data structure as a stack to avoid recursion.
     * Implementing it any other way WILL cause you to lose points!
     *
     * You may import/use {@code java.util.Set}, {@code java.util.Map},
     * {@code java.util.List}, and any classes that implement the
     * aforementioned interfaces.
     *
     * @throws IllegalArgumentException if any input is null, or if
     *         {@code start} doesn't exist in the graph
     * @param start the Vertex you are starting at
     * @param graph the Graph we are searching
     * @param <T> the data type representing the vertices in the graph.
     * @return a List of vertices in the order that you visited them
     */
    public static <T> List<Vertex<T>> depthFirstSearch(Vertex<T> start,
                                                       Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("One or more inputs was null!");
        }
        List<Vertex<T>> known = new ArrayList<>();
        DFS(graph, start, known);
        return known;
    }

    private static <T> void DFS(Graph<T> g, Vertex<T> u, List<Vertex<T>> known) {

        known.add(u);
        for (Vertex<T> v : outGoingVertices(g, u)) {
            if (!known.contains(v)) {
                DFS(g, v, known);
            }
        }

    }

    private static <T> List<Vertex<T>> outGoingVertices(Graph<T> g, Vertex<T> u) {
        List<Vertex<T>> list = new ArrayList<>();
        for (VertexDistancePair<T> v : g.getAdjacencyList().get(u)) {
            list.add(v.getVertex());
        }
        return list;
    }

    /**
     * Find the shortest distance between the start vertex and all other
     * vertices given a weighted graph where the edges only have positive
     * weights.
     *
     * Return a map of the shortest distances such that the key of each entry is
     * a node in the graph and the value for the key is the shortest distance
     * to that node from start, or Integer.MAX_VALUE (representing infinity)
     * if no path exists. You may assume that going from a vertex to itself
     * has a distance of 0.
     *
     * There are guaranteed to be no negative edge weights in the graph.
     * The graph passed in may be directed or undirected, but never both.
     *
     * You may import/use {@code java.util.PriorityQueue},
     * {@code java.util.Map}, and any class that implements the aforementioned
     * interface.
     *
     * @throws IllegalArgumentException if any input is null, or if
     *         {@code start} doesn't exist in the graph
     * @param start the Vertex you are starting at
     * @param graph the Graph we are searching
     * @param <T> the data type representing the vertices in the graph.
     * @return a map of the shortest distances from start to every other node
     *         in the graph.
     */
    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start,
                                                        Graph<T> graph) {
        Map<Vertex<T>, List<VertexDistancePair<T>>> adList = graph.getAdjacencyList();
        Map<Vertex<T>, Integer> d = new HashMap<>();
        Map<Vertex<T>, Integer> cloud = new HashMap<>();
        Map<Integer, Vertex<T>> pqHolder = new HashMap<>();
        Queue<Integer> pq = new PriorityQueue<>();
        Map<Vertex<T>, Map.Entry<Integer, Vertex<T>>> pqTokens = new HashMap<>();
        for (Vertex<T> v : adList.keySet()) {
            if (v == start) {
                d.put(v, 0);
            } else {
                d.put(v, Integer.MAX_VALUE);
            }
            int key = d.get(v);
            pq.add(key);
            pqHolder.put(key, v);
        }

        while(!pq.isEmpty()) {
            int key = pq.poll();
            Vertex<T> u = pqHolder.get(key);
            cloud.put(u, key);
            for (Vertex<T> v : outGoingVertices(graph, u)) {
                if(cloud.get(v) == null) {
                    int wgt =
                }
            }
        }

    }

    private static <T> List<VertexDistancePair<T>> getMapOfVerticesAndDistances(Graph<T> g, Vertex<T> s) {
        List<VertexDistancePair<T>> result = new ArrayList<>();
        for (VertexDistancePair<T> d : g.getAdjacencyList().get(s)) {
            result.add(d);
        }
        return result;
    }

    /**
     * Run Kruskal's algorithm on the given graph and return the minimum
     * spanning tree in the form of a set of Edges. If the graph is
     * disconnected, and therefore there is no valid MST, return null.
     *
     * You may assume that there will only be one valid MST that can be formed.
     * In addition, only an undirected graph will be passed in.
     *
     * You may import/use {@code java.util.PriorityQueue},
     * {@code java.util.Set}, {@code java.util.Map} and any class from java.util
     * that implements the aforementioned interfaces.
     *
     * @throws IllegalArgumentException if graph is null
     * @param graph the Graph we are searching
     * @param <T> the data type representing the vertices in the graph.
     * @return the MST of the graph; null if no valid MST exists.
     */
    public static <T> Set<Edge<T>> kruskals(Graph<T> graph) {
        //TODO kruskals
        return null;
    }
}