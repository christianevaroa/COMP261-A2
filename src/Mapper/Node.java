package Mapper;

/* Code for COMP261 Assignment
 * Name:
 * Usercode:
 * ID:
 */

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.util.*;
import java.io.*;


/** Node   */

public class Node{

	private int id;  
	private Location loc;  // coordinates of the intersection
	private List<Segment> outNeighbours = new ArrayList<Segment>(2);
	private List<Segment> inNeighbours = new ArrayList<Segment>(2);
	private boolean visited;
	private Node pathFrom;
	private double cost;
	private double depth;


	/** Construct a new Node object */
	public Node(int id, Location l){
		this.id = id;
		loc = l;
	}

	/** Construct a new Node object from a line in the data file*/
	public Node(String line){
		String[] values = line.split("\t");
		id = Integer.parseInt(values[0]);
		double lat = Double.parseDouble(values[1]);
		double lon = Double.parseDouble(values[2]);
		loc = Location.newFromLatLon(lat, lon);
		//System.out.printf("Created Node %6d %s%n", id, loc);
	}

	public int getID(){
		return id;
	}
	public Location getLoc(){
		return this.loc;
	}
	public void addInSegment(Segment seg){
		inNeighbours.add(seg);
	}	
	public void addOutSegment(Segment seg){
		outNeighbours.add(seg);
	}	

	public List<Segment> getOutNeighbours(){
		return outNeighbours;
	}	

	public List<Segment> getInNeighbours(){
		return inNeighbours;
	}	

	public boolean closeTo(Location place, double dist){
		return loc.closeTo(place, dist);
	}

	public double distanceTo(Location place){
		return loc.distanceTo(place);
	}

	public void draw(Graphics g, Location origin, double scale){
		Point p = loc.getPoint(origin, scale);
		g.fillRect(p.x, p.y, 2, 2);
	}

	public void drawAP(Graphics g, Location origin, double scale){
		Point p = loc.getPoint(origin, scale);
		g.fillOval(p.x-2, p.y-2, 4, 4);
	}

	public String toString(){
		StringBuilder b = new StringBuilder(String.format("Intersection %d: at %s; Roads:  ", id, loc));
		Set<String> roadNames = new HashSet<String>();
		for (Segment neigh : inNeighbours){
			roadNames.add(neigh.getRoad().getName());
		}
		for (Segment neigh : outNeighbours){
			roadNames.add(neigh.getRoad().getName());
		}
		for (String name : roadNames){
			b.append(name).append(", ");
		}
		return b.toString();
	}

	public boolean visited(){
		return this.visited;
	}

	public void visit(boolean b){
		this.visited = b;
	}

	public void setFrom(Node n){
		this.pathFrom = n;
	}

	public Node from(){
		return this.pathFrom;
	}

	public void setCost(double cost){
		this.cost = cost;
	}

	public double cost(){
		return this.cost;
	}

	public double depth(){
		return this.depth;
	}

	public void setDepth(double depth){
		this.depth = depth;
	}

	public List<Node> getNeighNodes(){
		List<Node> neighbours = new ArrayList<Node>(2);
		for(Segment s: outNeighbours){
			neighbours.add(s.getEndNode());
		}
		for(Segment s: inNeighbours){
			neighbours.add(s.getStartNode());
		}
		return neighbours;
	}


}
