package Mapper;

import java.util.ArrayDeque;

public class DFSNode {
	private Node node;
	private DFSNode parent;
	private double depth;
	private double reachback;
	private ArrayDeque<Node> children;
	
	public DFSNode(Node node, double depth, DFSNode parent){
		this.node = node;
		this.depth = depth;
		this.parent = parent;
	}
	
	public Node node(){
		return this.node;
	}
	
	public void setChildren(ArrayDeque<Node> children){
		this.children = children;
	}
	
	public ArrayDeque<Node> children(){
		return this.children;
	}
	
	public double depth(){
		return this.depth;
	}
	
	public double reach(){
		return this.reachback;
	}

	public void setReach(double reach) {
		this.reachback = reach;
	}
	
	public DFSNode parent(){
		return this.parent;
	}

}
