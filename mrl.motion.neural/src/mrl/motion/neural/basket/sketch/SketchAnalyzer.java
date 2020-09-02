package mrl.motion.neural.basket.sketch;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

import mrl.motion.neural.basket.sketch.SketchLine.LineType;

public class SketchAnalyzer {

	
	private SketchData sketch;
	private String[] nodeLabels;
	private HashMap<String, ArrayList<SketchNode>> nodeMap = new HashMap<String, ArrayList<SketchNode>>();
	private SketchNode startNode = null;

	public SketchAnalyzer(SketchData sketch) {
		this.sketch = sketch;
		
		HashSet<String> nodeIds = new HashSet<String>();
		for (SketchNode node : sketch.getNodeList()){
			nodeIds.add(node.label);
		}
		nodeLabels = nodeIds.toArray(new String[0]);
		Arrays.sort(nodeLabels);
		
		
		for (String label : nodeLabels) {
			ArrayList<SketchNode> sequence = getNodeSequence(label);
			nodeMap.put(label, sequence);
			if (sketch.getLine(sequence.get(0), sequence.get(1)).type() == LineType.Dribble){
				if (startNode != null) throw new RuntimeException();
				startNode = sequence.get(0);
			}
		}
	}
	
	private ArrayList<SketchNode> getNodeSequence(String label){
		HashSet<SketchNode> nodeSet = new HashSet<SketchNode>();
		for (SketchNode node : sketch.getNodeList()){
			if (node.label.equals(label)){
				nodeSet.add(node);
			}
		}
		for (SketchLine line : sketch.getLineList()){
			if (line.type() == LineType.Pass) continue;
			nodeSet.remove(line.target);
		}
		if (nodeSet.size() != 1){
			throw new RuntimeException("remain : " + nodeSet.size());
		}
		SketchNode startNode = nodeSet.iterator().next();
		ArrayList<SketchNode> sequence = new ArrayList<SketchNode>();
		while (startNode != null){
			sequence.add(startNode);
			startNode = getNextNode(startNode);
		}
		return sequence;
	}
	
	private SketchNode getPrevNode(SketchNode node){
		for (SketchLine line : sketch.getLineList()){
			if (line.type() == LineType.Pass) continue;
			if (line.target == node) return line.source;
		}
		return null;
	}
	
	private SketchNode getNextNode(SketchNode node){
		for (SketchLine line : sketch.getLineList()){
			if (line.type() == LineType.Pass) continue;
			if (line.source == node) return line.target;
		}
		return null;
	}
	
	private SketchLine getPassLine(SketchNode node){
		for (SketchLine line : sketch.getLineList()){
			if (line.type() == LineType.Pass && line.source == node) return line;
		}
		return null;
	}
	
	public ArrayList<BallTrajectory> getBallTrajectory(){
		SketchNode current = startNode;
		int currentTime = 0;
		ArrayList<BallTrajectory> list = new ArrayList<BallTrajectory>();
		while (true){
			SketchNode node1 = current;
			SketchNode node2;
			SketchLine passLine = getPassLine(current);
			if (passLine != null){
				node2 = node1;
			} else {
				node2 = getNextNode(node1);
				passLine = getPassLine(node2);
			}
			
			double moveDist = node1.position.distance(node2.position);
			int moveTime = 30 + (int)((moveDist/150)*30);
			
			
			int passTime = 0;
			SketchNode passedNode = null;
			if (passLine != null){
				double passDist = passLine.source.position.distance(passLine.target.position);
				passTime = 5 + (int)((passDist/150)*5);
				passedNode = passLine.target;
			}
			list.add(new BallTrajectory(node1, node2, passedNode, moveTime, passTime));
			
			if (passLine == null) break;
			
			currentTime = currentTime + moveTime + passTime;
			current = passLine.target;
		}
		return list;
	}
	
	
	public static class BallTrajectory{
		public SketchNode source;
		public SketchNode target;
		public SketchNode passedNode;
		public int moveTime;
		public int passTime;
		public BallTrajectory(SketchNode source, SketchNode target,
				SketchNode passedNode, int moveTime, int passTime) {
			this.source = source;
			this.target = target;
			this.passedNode = passedNode;
			this.moveTime = moveTime;
			this.passTime = passTime;
		}
		
	}
}
