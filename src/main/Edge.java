package main;

import lejos.robotics.pathfinding.Node;
import java.util.HashSet;
import java.util.Set;

public class Edge {
    private Set<Node> nodes;
    private int weight;

    public Edge(Node sourceNode, Node targetNode) {
        this.nodes = new HashSet<Node>();
        this.nodes.add(sourceNode);
        this.nodes.add(targetNode);
        this.weight = 0;
    }

    public Edge(Node sourceNode, Node targetNode, int weight) {
        this.nodes = new HashSet<Node>();
        this.nodes.add(sourceNode);
        this.nodes.add(targetNode);
        this.weight = weight;
    }

    public Set<Node> getNodes() {
        return this.nodes;
    }

    public Node[] getNodesArray() {
        Object[] objArr = nodes.toArray();
        Node[] nodesArr = new Node[2];
        nodesArr[0] = (Node) objArr[0];
        nodesArr[1] = (Node) objArr[1];
        return nodesArr;
    }

    public int getWeight() {
        return this.weight;
    }

    public void setWeight(int weight) {
        this.weight = weight;
    }

    public boolean hasNode(Node node) {
        return nodes.contains(node);
    }

    public boolean hasNodes(Node node1, Node node2) {
        return (nodes.contains(node1) && nodes.contains(node2));
    }

    public Node getLinkedNode(Node node) {
        Object[] nodesArray = nodes.toArray();
        if (nodesArray[0] == node) {
            return (Node) nodesArray[1];
        } else if (nodesArray[1] == node) {
            return (Node) nodesArray[0];
        }
        return null;
    }
}
