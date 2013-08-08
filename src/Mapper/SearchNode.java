package Mapper;

public class SearchNode implements Comparable {
	private Node node;
	private Node from;
	private boolean visited;
	private double costToHere;
	private double totalCostToGoal;
	
	public SearchNode(Node node, Node from, double costToHere, double estimate){
		this.node = node;
		this.from = from;
		this.costToHere = costToHere;
		this.totalCostToGoal = costToHere + estimate;
	}
	
	public boolean visited(){
		return this.visited;
	}
	
	public void visit(){
		this.visited = true;
	}

	@Override
	public int compareTo(Object o) {
		SearchNode other = (SearchNode)o;
		if(this.totalCostToGoal < other.totalCostToGoal){
			return 1;
		}
		else if(this.totalCostToGoal > other.totalCostToGoal){
			return -1;
		}
		return 0;
	}
}
