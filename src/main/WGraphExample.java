package main;

import java.util.List;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import lejos.robotics.pathfinding.Node;

public class WGraphExample {
	
	public WGraphExample() {
		SimpleWeightedGraph<Node, DefaultWeightedEdge>  weightedGraph = 
	            new SimpleWeightedGraph<Node, DefaultWeightedEdge>(DefaultWeightedEdge.class); 
		Node n1, n2, n3;
		DefaultWeightedEdge e1, e2, e3;
		
		n1 = new Node(1, 1);
		n2 = new Node(2, 2);
		n3 = new Node(3, 3);		
		
		weightedGraph.addVertex(n1);
		weightedGraph.addVertex(n2);
		weightedGraph.addVertex(n3);

		e1 = weightedGraph.addEdge(n1, n2);
		e2 = weightedGraph.addEdge(n2, n3);
		e3 = weightedGraph.addEdge(n3, n1);

		weightedGraph.setEdgeWeight(e1, 5);
		weightedGraph.setEdgeWeight(e2, 6);
		weightedGraph.setEdgeWeight(e3, 7);
		
        List shortestPath = (List) DijkstraShortestPath.findPathBetween(weightedGraph, n1, n3);
	}	
}
