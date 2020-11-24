package main;

import lejos.robotics.pathfinding.Node;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.HashSet;

public class WeightedGraph {
	HashMap<Node, List<Edge>> graph;

	public WeightedGraph() {
		this.graph = new HashMap<Node, List<Edge>>();
	}

	public void addNode(Node node) {
		//add node with empty edge list
		graph.put(node, new ArrayList<Edge>());
	}

	public void removeNode(Node nodeToDelete){
		//remove node and all edges to that node.
		List<Edge> edgesToDelete = graph.get(nodeToDelete);
		Set<Node> linkedNodes = getNeighbors(nodeToDelete);

		for (Node node : linkedNodes) {
			List<Edge> edges = graph.get(node);
			List<Edge> toRemove = new ArrayList<Edge>();
			for (Edge edge : edges) {
				if (edge.hasNode(nodeToDelete)) {
					toRemove.add(edge);
				}
			}
			edges.removeAll(toRemove);
		}
		graph.remove(nodeToDelete);
	}

	public Set<Node> getNeighbors(Node node) {
		//returns list of nodes with an edge to provided node
		Set<Node> linkedNodes = new HashSet<>();
		List<Edge> sourceNodeEdges = graph.get(node);
		for (Edge edge : sourceNodeEdges) {
			Node linkedNode = edge.getLinkedNode(node);
			linkedNodes.add(linkedNode);
		}
		return linkedNodes;
	}

	public List<Edge> getEdges(Node node) {
		return graph.get(node);
	}

	public Edge getEdge(Node node1, Node node2) {
		List<Edge> edges = graph.get(node1);
		for (Edge edge : edges) {
			if (edge.hasNodes(node1, node2)) {
				return edge;
			}
		}
		return null;
	}

	public void addEdge(Edge edge) {
		Node[] nodes = edge.getNodesArray();
		addEdge(nodes[0], nodes[1], edge);
	}

	public void addEdge(Node source, Node target, Edge e){
		if (!graph.containsKey(source)) {
			addNode(source);
		}
		if (!graph.containsKey(target)) {
			addNode(target);
		}
		List<Edge> sourceEdges, targetEdges;
		sourceEdges = graph.get(source);
		targetEdges = graph.get(target);
		sourceEdges.add(e);
		targetEdges.add(e);
	}

	public void addEdge(Node source, Node target, int weight){
		if (!graph.containsKey(source)) {
			addNode(source);
		}
		if (!graph.containsKey(target)) {
			addNode(target);
		}
		Edge edge = new Edge(source, target, weight);
		List<Edge> sourceEdges = graph.get(source);
		List<Edge> targetEdges = graph.get(target);
		sourceEdges.add(edge);
		targetEdges.add(edge);
	}
	public void removeEdge(Edge edgeToRemove) {
		Node[] nodes = edgeToRemove.getNodesArray();
		List<Edge> sourceNodeEdges = graph.get(nodes[0]);
		List<Edge> targetNodeEdges = graph.get(nodes[1]);
		sourceNodeEdges.remove(edgeToRemove);
		targetNodeEdges.remove(edgeToRemove);
	}
	public void removeEdge(Node node1, Node node2) {
		List<Edge> sourceNodeEdges = graph.get(node1);
		List<Edge> targetNodeEdges = graph.get(node2);
		Edge edge = getEdge(node1, node2);
		sourceNodeEdges.remove(edge);
		targetNodeEdges.remove(edge);
	}
	public Set<Node> getAllNodes() {
		return graph.keySet();
	}
	public void removeAllNodes() {
		graph.clear();
	}
	public Set<Edge> getAllEdges(){
		Set<Edge> edges = new HashSet<Edge>();
		for (Node node : graph.keySet()) {
			edges.addAll(graph.get(node));
		}
		return edges;
	}

	public void deleteAllEdges() {
		for (Node node : graph.keySet()) {
			List<Edge> edges = graph.get(node);
			edges.clear();
		}
	}

	public int size() {
		return this.graph.size();
	}
}
