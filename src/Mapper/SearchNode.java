package Mapper;

/* Code for COMP261 Assignment
 * Name:		Christian Evaroa
 * Usercode:	evaroachri
 * ID:			300276046

 */

public class SearchNode implements Comparable {
	private Node node;
	private Node from;
	private double costToHere;
	private double totalCostToGoal;
	
	public SearchNode(Node node, Node from, double costToHere, double estimate){
		this.node = node;
		this.from = from;
		this.costToHere = costToHere;
		this.totalCostToGoal = costToHere + estimate;
	}
	
	public Node node(){
		return this.node;
	}
	
	public Node from(){
		return this.from;
	}
	
	public double costToHere(){
		return costToHere;
	}
	
	public double totalCostToGoal(){
		return totalCostToGoal;
	}

	@Override
	public int compareTo(Object o) {
		SearchNode other = (SearchNode)o;
		if(this.totalCostToGoal < other.totalCostToGoal){
			return -1;
		}
		else if(this.totalCostToGoal > other.totalCostToGoal){
			return 1;
		}
		return 0;
	}
}
