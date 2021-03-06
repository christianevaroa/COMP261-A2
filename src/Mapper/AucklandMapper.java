package Mapper;

/* Code for COMP261 Assignment
 */


// call repaint() on this object to invoke the drawing.

import java.awt.Graphics;
import java.awt.Point;
import javax.swing.*;
import javax.swing.filechooser.FileFilter;
import java.awt.event.*;
import java.awt.BorderLayout;
import java.awt.GridBagLayout;
import java.awt.Color;
import java.util.*;
import java.io.*;


public class AucklandMapper{

	private JFrame frame;
	private JComponent drawing; 
	private JTextArea textOutput;
	private JTextField nameEntry;
	private int windowSize = 700;

	private RoadGraph roadGraph;

	private Node selectedNode;  // the currently selected node
	private Node destinationNode;	// the destination (find a path from selectedNode to destinationNode)
	private List<Segment> selectedSegments;  // the currently selected road or path
	private Set<Node> articulationPoints;

	private boolean loaded = false;
	private boolean distance = true;

	// Dimensions for drawing
	double westBoundary ;
	double eastBoundary ;
	double southBoundary;
	double northBoundary;
	Location origin;
	double scale;



	public AucklandMapper(String dataDir){
		setupInterface();
		roadGraph = new RoadGraph();

		textOutput.setText("Loading data...");
		while (dataDir==null){dataDir=getDataDir();}
		textOutput.append("Loading from "+dataDir+"\n");
		textOutput.append(roadGraph.loadData(dataDir));
		setupScaling();
		loaded = true;
		drawing.repaint();
	}

	private class DirectoryFileFilter extends FileFilter{
		public boolean accept(File f) {return f.isDirectory();}
		public String getDescription(){return "Directories only";}
	}

	private String getDataDir(){
		JFileChooser fc = new JFileChooser();
		fc.setFileFilter(new DirectoryFileFilter());
		fc.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
		if (fc.showOpenDialog(frame)!=JFileChooser.APPROVE_OPTION){return null;}
		return fc.getSelectedFile().getPath()+File.separator;
	}

	private void setupScaling(){
		double[] b = roadGraph.getBoundaries();
		westBoundary = b[0];
		eastBoundary = b[1];
		southBoundary = b[2];
		northBoundary = b[3];
		resetOrigin();
		/*
	      System.out.printf("Boundaries: w %.2f, e %.2f, s %.2f, n %.2f%n",
	      b[0], b[1], b[2], b[3]);
	      System.out.printf("Scaling from %s @ %.5f,%n", origin, scale);
		 */
	}

	private void setupInterface(){
		// Set up a window .
		frame = new JFrame("Graphics Example");
		frame.setSize(windowSize, windowSize);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		// Set up a JComponent in the window that we can draw on
		// When the JComponent tries to paint itself, it will call the redraw method
		//  in this PathDrawer class, passing a Graphics object to it.
		// The redraw method can draw whatever it wants on the Graphics object.
		// We can ask the JComponent to paint itself by calling drawing.repaint() 
		//  Note that this merely requests that the drawing is repainted; it won't
		//  necessarily do it immediately.
		drawing = new JComponent(){
			protected void paintComponent(Graphics g){redraw(g);}
		};
		frame.add(drawing, BorderLayout.CENTER);

		// Setup a text area for output
		textOutput = new JTextArea(5, 100);
		textOutput.setEditable(false);
		JScrollPane textSP = new JScrollPane(textOutput);
		frame.add(textSP, BorderLayout.SOUTH);

		//Set up a panel for some buttons.
		//To get nicer layout, we would need a LayoutManager on the panel.
		JPanel panel = new JPanel();
		frame.add(panel, BorderLayout.NORTH);

		//Add a text label to the panel.
		//	panel.add(new JLabel("Click to select"));

		// Add  buttons to the panel.
		JButton button = new JButton("+");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){zoomIn();}});

		button = new JButton("-");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){zoomOut();}});

		button = new JButton("<");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){pan("left");}});

		button = new JButton(">");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){pan("right");}});

		button = new JButton("^");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){pan("up");}});

		button = new JButton("v");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){pan("down");}});
		
		button = new JButton("Dist / Speed");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){
				distance = !distance;
				if(distance){
					textOutput.setText("Now finding shortest path");
				} else {
					textOutput.setText("Now finding fastest path");
				}
				}});

		button = new JButton("Deselect");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){
				selectedNode = null;
				destinationNode = null;
				selectedSegments = null;
				articulationPoints = null;
				drawing.repaint();}});

		button = new JButton("A. Pts");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){
				if(selectedNode != null){
					textOutput.setText("Finding articulation points...");
					for(Node n: selectedNode.getNeighNodes()){
						articulationPoints = new HashSet<Node>();
						articulationPoints.addAll(roadGraph.iterDFS(n, 0, new DFSNode(selectedNode, 0, null)));
						drawing.repaint();
					}
					for(Node n : roadGraph.nodes().values()){
						if(!n.visited()){
							articulationPoints.addAll(roadGraph.iterDFS(n, 0, new DFSNode(n, 0, null)));
						}
					}
					textOutput.setText("Found "+articulationPoints.size()+" articulation points.");
				}
				else{
					textOutput.setText("You must select a node to find articulation points");
				}
				}});


		nameEntry = new JTextField(20);
		panel.add(nameEntry);
		nameEntry.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent e){
				lookupName(nameEntry.getText());
				drawing.repaint();
			}});

		button = new JButton("Quit");
		panel.add(button);
		button.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ev){System.exit(0);}});

		// Add a mouselistener to the drawing JComponent to respond to mouse clicks.
		drawing.addMouseListener(new MouseAdapter(){
			public void mouseReleased(MouseEvent e){
				Node tempNode = findNode(e.getPoint());
				if (selectedNode==null){
					selectedNode = findNode(e.getPoint());
					textOutput.setText(selectedNode.toString());
				}
				else if (selectedNode != null){
					destinationNode = findNode(e.getPoint());
					selectedSegments = roadGraph.findPath(selectedNode, destinationNode, distance);
					if(selectedSegments == null){ textOutput.setText("No path between these nodes."); return; }
					textOutput.setText("Path from "+selectedNode.toString()+" to "+destinationNode.toString()+": ");
					ArrayList<String> roadNames = new ArrayList<String>();
					HashMap<Integer, Double> roadNamesLengths = new HashMap<Integer, Double>();
					int i = 0;
					for(Segment s : selectedSegments){
						String road = s.getRoad().getName();
						double length = s.getLength();
						if(i >= roadNames.size()){
							roadNames.add(road);
						}
						if(!roadNames.get(i).equals(road)){
							roadNames.add(road);
							i++;
						}
						if(!roadNamesLengths.containsKey(i)){
							roadNamesLengths.put(i, length);
						}
						else{
							roadNamesLengths.put(i, roadNamesLengths.get(i)+length);
						}
					}
					double totalLength = 0;
					for(i = 0; i < roadNames.size(); i++){
						totalLength += roadNamesLengths.get(i);
						textOutput.append("\n"+roadNames.get(i)+": "+roadNamesLengths.get(i)+" km");
					}
					textOutput.append("\nTotal distance: "+totalLength+" km");
				}
				drawing.repaint();}});

		// Once it is all set up, make the interface visible
		frame.setVisible(true);

	}
	private double zoomFactor = 1.25;  
	private double panFraction = 0.2;  

	//set origin and scale for the whole map
	private void resetOrigin(){
		origin = new Location(westBoundary, northBoundary);
		scale = Math.min(windowSize/(eastBoundary-westBoundary),
				windowSize/(northBoundary-southBoundary));
	}

	//shrink the scale (pixels/per km) by zoomFactor and move origin 
	private void zoomOut(){
		scale =  scale/zoomFactor;
		double deltaOrig = windowSize/scale*(zoomFactor-1)/zoomFactor/2;
		origin = new Location(origin.x - deltaOrig, origin.y + deltaOrig);
		drawing.repaint();
	}
	//expand the scale (pixels/per km) by zoomFactor and move origin 
	private void zoomIn(){
		double deltaOrig = windowSize/scale*(zoomFactor-1)/zoomFactor/2;
		origin = new Location(origin.x + deltaOrig, origin.y - deltaOrig);
		scale =  scale*zoomFactor;
		drawing.repaint();	    
	}

	private void pan(String dir){
		double delta = windowSize*panFraction/scale;
		if(dir.equals("left")){ origin = new Location(origin.x-delta, origin.y); }
		else if(dir.equals("right")){ origin = new Location(origin.x+delta, origin.y); }
		else if(dir.equals("up")){ origin = new Location(origin.x, origin.y+delta); }
		else if(dir.equals("down")){ origin = new Location(origin.x, origin.y-delta); }
		drawing.repaint();	    
	}
	//Find the place that the mouse was clicked on (if any)
	private Node findNode(Point mouse){
		return roadGraph.findNode(mouse, origin, scale);
	}


	private void lookupName(String query){
		List<String> names = new ArrayList(roadGraph.lookupName(query));
		if (names.isEmpty()){
			selectedSegments=null;
			textOutput.setText("Not found");
		}
		else if (names.size()==1){
			String fullName =names.get(0);
			nameEntry.setText(fullName);
			textOutput.setText("Found");
			selectedSegments=roadGraph.getRoadSegments(fullName);
		}
		else{
			selectedSegments=null;
			String prefix = maxCommonPrefix(query, names);
			nameEntry.setText(prefix);
			textOutput.setText("Options: ");
			for (int i = 0; i<10&&i<names.size(); i++){
				textOutput.append(names.get(i));textOutput.append(", ");
			}
			if (names.size()>10){textOutput.append("...\n");}
			else { textOutput.append("\n"); }
		}
	}

	private String maxCommonPrefix(String query, List<String>names){
		String ans = query;
		for (int i = query.length(); ; i++){
			if (names.get(0).length()<i) return ans;
			String cand = names.get(0).substring(0,i);
			for (String name : names){
				if (name.length()<i) return ans;
				if (name.charAt(i-1)!=cand.charAt(i-1)) return ans;
			}
			ans = cand;
		}
	}



	//The redraw method that will be called from the drawing JComponent and will
	//draw the map at the current scale and shift.
	public void redraw(Graphics g){
		if (roadGraph!= null && loaded ){
			roadGraph.redraw(g, origin, scale);
			if (selectedNode!=null) {
				g.setColor(Color.red);
				selectedNode.draw(g, origin, scale);}
			if(destinationNode!=null) {
				g.setColor(Color.red);
				destinationNode.draw(g, origin, scale);}
		}
		if (selectedSegments!=null){
			g.setColor(Color.red);
			for (Segment seg : selectedSegments){
				seg.draw(g, origin, scale);
			}
		}
		if(articulationPoints != null){
			g.setColor(Color.green);
			for(Node n : articulationPoints){
				n.drawAP(g, origin, scale);
			}
		}
	}


	public static void main(String[] arguments){
		if (arguments.length>0){
			AucklandMapper obj = new AucklandMapper(arguments[0]);
		}
		else{
			AucklandMapper obj = new AucklandMapper(null);
		}
	}	


}
