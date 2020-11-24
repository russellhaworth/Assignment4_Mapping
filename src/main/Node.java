package main;

public class Node {
	private int x;
	private int y;
	private boolean collisionNode;
	private boolean obstacle;
	
	public Node(int x, int y, boolean collisionNode, boolean obstacle) {
		this.x = x;
		this.y = y;
		this.collisionNode = collisionNode;
		this.obstacle = obstacle;
	}
	
	public void setX(int x) {
		this.x= x; 
	}
	
	public void setY(int y) {
		this.y = y; 
	}
	
	public int getX() {
		return this.x;
	}
	
	public int getY() {
		return this.y;
	}
	
	public boolean getCollisionNode() {
		return this.collisionNode;
	}
	
	public void setCollisionNode(boolean b) {
		this.collisionNode = b;
	}
	
	public boolean getObstalceStatus() {
		return this.obstacle;
	}
	
	public void setObstacleStatus(boolean b) {
		this.obstacle = b;
	}
}