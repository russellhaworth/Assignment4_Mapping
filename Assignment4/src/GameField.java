import java.awt.List;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class GameField {
	int boardLength = 142;
	int boardWidth = 114;
	ArrayList<Node> visitedNodes = new ArrayList<Node>();
	ArrayList<Node> nodesToVisit = new ArrayList<Node>();
	Node[][] gameField = new Node[boardLength][boardWidth];
	
	GameField(){
		for(int i = 0; i < gameField.length; i++) {
			for(int j = 0; j < gameField[i].length; j++) {
				gameField[i][j] = new Node(i,j,false,false);
			}
		}
	}
	
	//important: need to overload Node.equals(Node).
		//ArrayList<Node>.contains(Node n) may not work.
		public ArrayList<Node> BFS(Node startNode, Node endNode) {
			//list of nodes visited in search.
			ArrayList<Node> visited = new ArrayList<Node>();		
			
			//queue of generated paths.
			Queue queue = new LinkedList();

			//adds starting node to queue and visited.
			ArrayList<Node> initPath = new ArrayList<Node>();		
			initPath.add(startNode);
			queue.add(initPath);
			visited.add(startNode);
			
			while (!queue.isEmpty()) {
				//remove the next path from queue
				ArrayList<Node> currentPath = (ArrayList<Node>) queue.remove();
				
				//find the last node on the path
				Node currentNode = currentPath.get(-1);
				
				//get list of neighboring nodes.
				ArrayList<Node> neighbors = getNeighbors(currentNode);
				for (int i = 0; i<neighbors.size(); i++) {
					Node neighborNode = neighbors.get(i);

					//checks to see if the node has a chance for collision.
					if (neighborNode.getCollisionNode() == false) {
						//if neighbor == endNode, path found.
						if (neighborNode.equals(endNode)) {
							currentPath.add(neighborNode);
							return currentPath;
						}				
						
						//if neighbor not in visited: clone the current path, add neighbor to end, and add the path to the queue.
						if (!visited.contains(neighborNode)) {
							ArrayList<Node> newPath = new ArrayList<>(currentPath);
							newPath.add(neighborNode);
							visited.add(neighborNode);
							queue.add(newPath);
						}
					}
					
				}
			}
			//path not found, return empty list.
			return new ArrayList<Node>();
		}
		
		public ArrayList<Node> getNeighbors(Node node) {
			//removes the need of an undirected graph.  Uses x,y coordinates to find neighboring nodes.		
			Node sNode, nNode, eNode, wNode;
			//set to appropriate height/width.
			int boardWidth = 100;
			int boardHeight = 100;
			int nodeX = node.getX();
			int nodeY = node.getY();
			ArrayList<Node> neighbors = new ArrayList<Node>();
			//south neighbor
			if (nodeY > 0) {
				sNode = gameField[nodeX][nodeY-1];
				neighbors.add(sNode);
			}
			//north neighbor
			if (nodeY < boardHeight) {
				nNode = gameField[nodeX][nodeY+1];		
				neighbors.add(nNode);	
			}
			//west neighbor
			if (nodeX > 0) {
				wNode = gameField[nodeX-1][nodeY];	
				neighbors.add(wNode);
			}
			//east neighbor
			if (nodeX < boardWidth) {
				eNode = gameField[nodeX+1][nodeY];	
				neighbors.add(eNode);		
			}
			return neighbors;
		}
		
		public void markCollisionNodes(Node node) {
			//mar
			int robotSize = 40;
			int xMin = node.getX() - robotSize;
			int xMax = node.getX() + robotSize;
			int yMin = node.getY() - robotSize;
			int yMax = node.getY() + robotSize;
			
			for (int i = xMin; i <= xMax; i++) {
				for (int j = yMin; j <= yMax; j++) {
					Node collisionNode = gameField[i][j];
					collisionNode.setCollisionNode(true);
				}
			}
		}
		
		public boolean isClearPath(Node start, Node end) {
			ArrayList<Node> list = new ArrayList<Node>();
			float m = (end.getY() - start.getY())/ (end.getX() - start.getX());
			if (end.getX() > start.getX()) {
				for (int i = start.getX(); i <= end.getX(); i++) {
					float y = m * i;
					Node n = gameField[i][(int) Math.floor(y)];
					list.add(n);
				}
			} else {
				for (int i = start.getX(); i >= end.getX(); i--) {
					float y = m * i;
					Node n = gameField[i][(int) Math.floor(y)];
					list.add(n);				
				}			
			}
			if (end.getY() > start.getY()) {
				for (int j = start.getY(); j <= end.getY(); j++) {
					float x = j / m;
					Node n = gameField[(int) Math.floor(x)][j];
					list.add(n);		
				}
			} else {
				for (int j = start.getY(); j >= end.getY(); j--) {
					float x = j / m;
					Node n = gameField[(int) Math.floor(x)][j];
					list.add(n);						
				}			
			}
			for (int i = 0; i<list.size();i++) {
				Node n = list.get(i);
				if (n.getCollisionNode()) return false;
			}
			return true;		
		}
	
	

}
