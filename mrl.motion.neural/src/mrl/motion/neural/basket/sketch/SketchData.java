package mrl.motion.neural.basket.sketch;

import java.util.ArrayList;

public class SketchData {
	
	private ArrayList<SketchNode> nodeList = new ArrayList<SketchNode>();
	private ArrayList<SketchLine> lineList = new ArrayList<SketchLine>();
	
	public SketchData(){
		
	}

	public ArrayList<SketchNode> getNodeList() {
		return nodeList;
	}
	
	public ArrayList<SketchLine> getLineList() {
		return lineList;
	}

	public SketchNode newNode(String label){
		SketchNode node = new SketchNode();
		node.label = label;
		nodeList.add(node);
		return node;
	}
	
	public SketchLine newLine(){
		SketchLine line = new SketchLine();
		lineList.add(line);
		return line;
	}
	
	public void removeLine(SketchLine line){
		lineList.remove(line);
	}
	
	public void deleteNode(SketchNode node){
		nodeList.remove(node);
		SketchLine[] lines = lineList.toArray(new SketchLine[lineList.size()]);
		for (SketchLine line : lines){
			if (line.source == node || line.target == node){
				lineList.remove(line);
			}
		}
	}
	
	public void deleteLine(SketchLine line){
		lineList.remove(line);
	}
	
	public SketchLine getLine(SketchNode source, SketchNode target){
		for (SketchLine line : lineList){
			if (line.source == source && line.target == target) return line;
		}
		return null;
	}
	
}
