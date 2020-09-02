package mrl.motion.neural.predict;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.sketch.SketchData;
import mrl.motion.neural.basket.sketch.SketchLine;
import mrl.motion.neural.basket.sketch.SketchLine.LineType;
import mrl.motion.neural.basket.sketch.SketchNode;
import mrl.motion.neural.basket.sketch.SketchTexture;
import mrl.util.MathUtil;

public class SketchToPredicter {

	private SketchData sketchData;
	public ArrayList<PersonPredicter> personList;

	public SketchToPredicter(SketchData sketchData, int[][] fixedTime) {
		this.sketchData = sketchData;
		personList = new ArrayList<PersonPredicter>();
		
		ArrayList<SketchNode> nodeList = sketchData.getNodeList();
		HashSet<String> nodeIds = new HashSet<String>();
		for (SketchNode node : nodeList){
			nodeIds.add(node.label);
		}
		String[] nodeLabels = nodeIds.toArray(new String[0]);
		Arrays.sort(nodeLabels);
		System.out.println("person ids : " + Arrays.toString(nodeLabels));
		
		personList = new ArrayList<PersonPredicter>();
		HashMap<String, PersonPredicter> personMap = new HashMap<String, PersonPredicter>();
		HashMap<PersonPredicter, HashSet<SketchNode>> sNodeMap = new HashMap<PersonPredicter, HashSet<SketchNode>>();
		for (String label : nodeLabels){
			PersonPredicter person = new PersonPredicter(this, label);
			personList.add(person);
			personMap.put(label, person);
			sNodeMap.put(person, new HashSet<SketchNode>());
		}
		
		for (SketchNode node : nodeList){
			PersonPredicter person = personMap.get(node.label);
			HashSet<SketchNode> nodeSet = sNodeMap.get(person);
			nodeSet.add(node);
		}
		
		if (fixedTime != null){
			for (int i = 0; i < personList.size(); i++) {
				PersonPredicter p = personList.get(i);
				p.fixedTime = fixedTime[i];
			}
		}
		
		for (PersonPredicter person : personList){
			SketchNode sNode = getStartNode(sNodeMap.get(person));
			person.setStartNode(sNode);
		}
	}
	
	public SketchLine getPassLine(SketchNode node){
		for (SketchLine line : sketchData.getLineList()){
			if (line.type() == LineType.Pass && 
					(line.source == node || line.target == node)){
				return line;
			}
		}
		return null;
	}
	
	public SketchLine getOutgoingLine(SketchNode node){
		for (SketchLine line : sketchData.getLineList()){
			if (line.type() != LineType.Pass && line.type() != LineType.Screen && line.source == node){
				return line;
			}
		}
		return null;
	}
	
	private SketchNode getStartNode(HashSet<SketchNode> nodeSet){
		nodeSet = new HashSet<SketchNode>(nodeSet);
		for (SketchLine line : sketchData.getLineList()){
			if (line.type() == LineType.Pass) continue;
			nodeSet.remove(line.target);
		}
		if (nodeSet.size() != 1){
			throw new RuntimeException("remain : " + nodeSet.size());
		}
		return nodeSet.iterator().next();
	}
}
