package main;

public class Node extends lejos.robotics.pathfinding.Node {	
	private int weight;
	
	public Node(float x, float y) {
		super(x, y);
		// TODO Auto-generated constructor stub
	}
	
	public int getWeight() {
		return this.weight;
	}
	
	public void setWeight(int w) {
		this.weight = w;
	}
}
