package Mapper;

/* Code for COMP261 Assignment
 * Name:
 * Usercode:
 * ID:

 */

import java.util.*;
import java.io.*;
import java.awt.Color;
import java.awt.Point;
import java.awt.Graphics;

/** RoadMap: The list of the roads and the graph of the road network    */

public class RoadGraph{

	double westBoundary = Double.POSITIVE_INFINITY;
	double eastBoundary = Double.NEGATIVE_INFINITY;
	double southBoundary = Double.POSITIVE_INFINITY;
	double northBoundary = Double.NEGATIVE_INFINITY;


	//the map containing the graph of nodes (and roadsegments), indexed by the nodeID
	Map<Integer,Node> nodes = new HashMap<Integer,Node>();

	//the map of roads, indexed by the roadID
	Map<Integer,Road> roads = new HashMap<Integer,Road>();;

	//the map of roads, indexed by name
	Map<String,Set<Road>> roadsByName = new HashMap<String,Set<Road>>();;

	Set<String> roadNames = new HashSet<String>();

	/** Construct a new RoadMap object */
	public RoadGraph(){
	}

	public String loadData(String dataDirectory){
		// Read roads into roads array.
		// Read the nodes into the roadGraph array.
		// Read each road segment
		//   put the segment into the neighbours of the startNode
		//   If the road of the segment is not one way,
		//   also construct the reversed segment and put it into 
		//   the neighbours of the endNode
		// Work out the boundaries of the region.
		String report= "";
		System.out.println("Loading roads...");
		loadRoads(dataDirectory);
		report += String.format("Loaded %,d roads, with %,d distinct road names%n",
				roads.entrySet().size(), roadNames.size());
		System.out.println("Loading intersections...");
		loadNodes(dataDirectory);
		report += String.format("Loaded %,d intersections%n", nodes.entrySet().size());
		System.out.println("Loading road segments...");
		loadSegments(dataDirectory);
		report += String.format("Loaded %,d road segments%n", numSegments());
		return report;
	}


	public void loadRoads(String dataDirectory){
		File roadFile = new File(dataDirectory+"roadID-roadInfo.tab");
		if ( !roadFile.exists() ){
			System.out.println("roadID-roadInfo.tab not found");
			return;
		}
		BufferedReader data;
		try{
			data = new BufferedReader(new FileReader(roadFile));
			data.readLine(); //throw away header line.
			while (true){
				String line = data.readLine();
				if (line==null) {break;}
				Road road = new Road(line);
				roads.put(road.getID(), road);
				String fullName = road.getFullName();
				roadNames.add(fullName);
				Set<Road> rds = roadsByName.get(fullName);
				if (rds==null){
					rds = new HashSet<Road>(4);
					roadsByName.put(fullName, rds);
				}
				rds.add(road);
			}
		} catch(IOException e){System.out.println("Failed to open roadID-roadInfo.tab: " + e);}
	}

	public void loadNodes(String dataDirectory){
		File nodeFile = new File(dataDirectory+"nodeID-lat-lon.tab");
		if ( !nodeFile.exists() ){
			System.out.println("nodeID-lat-lon.tab not found");
			return;
		}
		BufferedReader data;
		try{
			data = new BufferedReader(new FileReader(nodeFile));
			while (true){
				String line = data.readLine();
				if (line==null) { break; }
				Node node = new Node(line);
				nodes.put(node.getID(), node);
			}
		} catch(IOException e){System.out.println("Failed to open roadID-roadInfo.tab: " + e);}
	}

	public void loadSegments(String dataDirectory){
		File segFile = new File(dataDirectory+"roadSeg-roadID-length-nodeID-nodeID-coords.tab");
		if ( !segFile.exists() ){
			System.out.println("roadSeg-roadID-length-nodeID-nodeID-coords.tab not found");
			return;
		}
		BufferedReader data;
		try{
			data = new BufferedReader(new  FileReader(segFile));
			data.readLine();  // get rid of headers
			while (true){
				String line = data.readLine();
				if (line==null) { break; }
				Segment seg = new Segment(line, roads, nodes);
				//System.out.println(seg);
				Node node1 = seg.getStartNode();
				Node node2 = seg.getEndNode();
				node1.addOutSegment(seg);
				node2.addInSegment(seg);
				Road road = seg.getRoad();
				road.addSegment(seg);
				if (!road.isOneWay()){
					Segment revSeg = seg.reverse();
					node2.addOutSegment(revSeg);
					node1.addInSegment(revSeg);
				}
			}
		} catch(IOException e){System.out.println("Failed to open roadID-roadInfo.tab: " + e);}
	}


	public double[] getBoundaries(){
		double west = Double.POSITIVE_INFINITY;
		double east = Double.NEGATIVE_INFINITY;
		double south = Double.POSITIVE_INFINITY;
		double north = Double.NEGATIVE_INFINITY;

		for (Node node : nodes.values()){
			Location loc = node.getLoc();
			if (loc.x < west) {west = loc.x;}
			if (loc.x > east) {east = loc.x;}
			if (loc.y < south) {south = loc.y;}
			if (loc.y > north) {north = loc.y;}
		}
		return new double[]{west, east, south, north};
	}

	public void checkNodes(){
		for (Node node : nodes.values()){
			if (node.getOutNeighbours().isEmpty()&& node.getInNeighbours().isEmpty()){
				System.out.println("Orphan: "+node);
			}
		}
	}

	public int numSegments(){
		int ans = 0;
		for (Node node : nodes.values()){
			ans += node.getOutNeighbours().size();
		}
		return ans;
	}



	public void redraw(Graphics g, Location origin, double scale){
		//System.out.printf("Drawing road graph. at (%.2f, %.2f) @ %.3f%n", origX, origY, scale);
		g.setColor(Color.black);
		for (Node node : nodes.values()){
			for (Segment seg : node.getOutNeighbours()){
				seg.draw(g, origin, scale);
			}
		}
		g.setColor(Color.blue);
		for(Node node : nodes.values()){
			node.draw(g, origin, scale);
		}
	}


	private double mouseThreshold = 5;  //how close does the mouse have to be?

	public Node findNode(Point point, Location origin, double scale){
		Location mousePlace = Location.newFromPoint(point, origin, scale);
		/* System.out.printf("find at %d %d -> %.3f %.3f -> %d %d %n",
	   point.x, point.y, x, y,
	   (int)((x-origX)*scale),(int)((y-origY)*(-scale)) );
		 */
		Node closestNode = null;
		double mindist = Double.POSITIVE_INFINITY;
		for (Node node : nodes.values()){
			double dist = node.distanceTo(mousePlace);
			if (dist<mindist){
				mindist = dist;
				closestNode = node;
			}
		}
		return closestNode;
	}

	/** Returns a set of full road names that match the query.
	 *  If the query matches a full road name exactly, then it returns just that name*/
	public Set<String> lookupName(String query){
		Set<String> ans = new HashSet<String>(10);
		if (query==null) return null;
		query = query.toLowerCase();
		for (String name : roadNames){
			if (name.equals(query)){  // this is the right answer
				ans.clear();
				ans.add(name);
				return ans;
			}
			if (name.startsWith(query)){ // it is an option
				ans.add(name);
			}
		}
		return ans; 
	}

	/** Get the Road objects associated with a (full) road name */
	public Set<Road> getRoadsByName(String fullname){
		return roadsByName.get(fullname);
	}

	/** Return a list of all the segments belonging to the road with the
     given (full) name. */
	public List<Segment> getRoadSegments(String fullname){
		Set<Road> rds = roadsByName.get(fullname);
		if (rds==null) { return null; }
		System.out.println("Found "+rds.size()+" road objects: "+rds.iterator().next());
		List<Segment> ans = new ArrayList<Segment>();
		for (Road road : rds){
			ans.addAll(road.getSegments());
		}
		return ans;
	}

	public static void main(String[] arguments){
		AucklandMapper.main(arguments);
	}

	/**
	 * Use A* algorithm to find the shortest path between two nodes.
	 * @param origin	The starting point.
	 * @param goal		The finishing point.
	 * @return List of Sections that makes up the shortest path from origin to goal.
	 */
	public List<Segment> findPath(Node origin, Node goal, boolean distance) {
		for(Node n : nodes.values()){
			n.visit(false);
		}
		List<Segment> path = new ArrayList<Segment>();
		PriorityQueue<SearchNode> fringe = new PriorityQueue<SearchNode>();
		Location goalLoc = goal.getLoc();
		SearchNode start = new SearchNode(origin, null, 0, origin.distanceTo(goalLoc));
		fringe.offer(start);
		while(!fringe.isEmpty()){
			SearchNode currentNode = fringe.poll();
			Node node = currentNode.node();
			if(!node.visited()){
				node.visit(true);
				node.setFrom(currentNode.from());
				node.setCost(currentNode.costToHere());
				if(node == goal){
					break;
				}
				for(Segment s : node.getOutNeighbours()){
					if(!s.getRoad().isNotForCars()){
						Node neigh = s.getEndNode();
						if(!neigh.visited()){
							double costToNeigh = currentNode.costToHere() + s.getLength();
							double estTotal = costToNeigh + neigh.distanceTo(goalLoc);
							if(!distance){
								switch(s.getRoad().getSpeed()){
								case(0):
									estTotal = estTotal-(estTotal*0.05);
									break;
								case(1):
									estTotal = estTotal-(estTotal*0.20);
									break;
								case(2):
									estTotal = estTotal-(estTotal*0.40);
									break;
								case(3):
									estTotal = estTotal-(estTotal*0.60);
									break;
								case(4):
									estTotal = estTotal-(estTotal*0.80);
									break;
								case(5):
									estTotal = estTotal-(estTotal*0.99);
									break;
								}
							}
							SearchNode newSearchNode = new SearchNode(neigh, node, costToNeigh, estTotal);
							fringe.offer(newSearchNode);
						}
					}
				}
			}
		}
		Node backTrack = goal;
		while(backTrack != origin){
			for(Segment s : backTrack.getInNeighbours()){
				if(s.getStartNode() == backTrack.from()){
					path.add(s);
					backTrack = backTrack.from();
					break;
				}
			}
		}
		Collections.reverse(path);
		return path;
	}	

	public Set<Node> iterDFS(Node firstNode, double d, DFSNode root){
		// initialise required collections
		HashSet<Node> articulationPoints = new HashSet<Node>();
		Stack<DFSNode> stack = new Stack<DFSNode>();
		// init all nodes (depth = +inf, visited = false)
		for(Node n : nodes.values()){
			n.visit(false);
			n.setDepth(Double.POSITIVE_INFINITY);
		}
		DFSNode first = new DFSNode(firstNode, 1, root);
		stack.push(first);
		while(!stack.isEmpty()){
			DFSNode elem = stack.peek();
			Node node = elem.node();
			node.visit(true);
			if(elem.children() == null){
				node.setDepth(elem.depth());
				elem.setReach(elem.depth());
				ArrayDeque<Node> children = new ArrayDeque<Node>(2);
				elem.setChildren(children);
				for(Node neighbour : node.getNeighNodes()){ // for each neighbour of node
					if(neighbour!=elem.parent().node()){	// if neighbour isnt parent
						children.offer(neighbour);			// add neighbour to children
					}
				}
			}
			else if(!elem.children().isEmpty()){
				Node child = elem.children().poll();
				if(child.depth() < Double.POSITIVE_INFINITY)
					elem.setReach(Math.min(elem.reach(), child.depth()));
				else
					stack.push(new DFSNode(child, node.depth()+1, elem));
			}
			else{
				if(node != firstNode){
					if(elem.reach() >= elem.parent().depth()){
						articulationPoints.add(elem.parent().node());
					}
					elem.parent().setReach(Math.min(elem.parent().reach(), elem.reach()));
				}
				stack.pop();	
			}
		}
		return articulationPoints;
	}

	public Map<Integer, Node> nodes(){
		return this.nodes;
	}

}