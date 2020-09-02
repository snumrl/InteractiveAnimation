package mrl.motion.graph;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionAnnotation;
import mrl.util.Configuration;

public class MGraph {
	
	private MDatabase database;
	private MGraphNode[] nodeList;
	private MGraphNode[] motionToNode;
	private MGraphEdge[][] edgeList;
	private MGraphEdge[][] transposedEdgeList;
	private MGraphEdge[] adjacentEdgeList;
	private MGraphEdge[] transposedAdjacentEdgeList;
	
	private HashMap<MotionAnnotation, MotionClip[]> clipMapping;
	
	public static MGraph load(){
		return new MGraph(MDatabase.load());
	}
	
	public MGraph(MDatabase database) {
		this.database = database;
		
		nodeList = loadNodeList();
		edgeList = loadEdgeList();
		
		transposedEdgeList = new MGraphEdge[nodeList.length][];
		@SuppressWarnings("unchecked")
		ArrayList<MGraphEdge>[] tEdgeList = (ArrayList<MGraphEdge>[])new ArrayList<?>[nodeList.length];
		for (int i = 0; i < tEdgeList.length; i++) tEdgeList[i] = new ArrayList<MGraphEdge>();
		for (MGraphEdge[] edges : edgeList){
			for (MGraphEdge edge : edges){
				tEdgeList[edge.target.index].add(edge);
			}
		}
		for (int i = 0; i < tEdgeList.length; i++){
			transposedEdgeList[i] = tEdgeList[i].toArray(new MGraphEdge[tEdgeList[i].size()]);
		}
		
		adjacentEdgeList = new MGraphEdge[nodeList.length];
		transposedAdjacentEdgeList = new MGraphEdge[nodeList.length];
		for (int i = 0; i < edgeList.length; i++) {
			MGraphEdge[] edges = edgeList[i];
			for (int j = 0; j < edges.length; j++) {
				if (edges[j].transition == 0){
					adjacentEdgeList[i] = edges[j];
					transposedAdjacentEdgeList[edges[j].target.index] = edges[j]; 
					break;
				}
			}
		}
		
//		clipMapping = MotionClip.getClipMapping(this, database.getEventAnnotations());
	}
	
	public MotionClip[] getClip(MotionAnnotation ann){
		return clipMapping.get(ann);
	}
	
	public MotionAnnotation getAnnotation(MotionClip clip){
		for (Entry<MotionAnnotation, MotionClip[]> entry : clipMapping.entrySet()){
			MotionClip[] clips = entry.getValue();
			for (int i = 0; i < clips.length; i++) {
				if (clips[i] == clip){
					return entry.getKey();
				}
			}
		}
		return null;
	}
	
	public MotionClip getClipByStartMotionIndex(int motionIndex){
		for (MotionClip[] clipList : clipMapping.values()){
			for (MotionClip clip : clipList){
				if (clip != null && clip.startMotionIndex == motionIndex){
					return clip;
				}
			}
		}
		return null;
	}
	
	public ArrayList<MotionClip[]> getMotionClips(String type){
		ArrayList<MotionAnnotation> annList = MotionAnnotation.filter(database.getEventAnnotations(), type, "*");
		ArrayList<MotionClip[]> list = new ArrayList<MotionClip[]>();
		for (MotionAnnotation ann : annList){
			if (!ann.isAction()) continue;
			list.add(clipMapping.get(ann));
		}
		return list;
	}
	
	public MDatabase getDatabase() {
		return database;
	}

	public MGraphNode[] getNodeList() {
		return nodeList;
	}

	public MGraphEdge[][] getEdgeList() {
		return edgeList;
	}
	
	public MGraphEdge getEdge(int source, int target){
		for (MGraphEdge edge : edgeList[source]){
			if (edge.target.index == target){
				return edge;
			}
		}
		return null;
	}
	
	public MGraphNode getNodeByMotion(Motion motion){
		return getNodeByMotion(motion.motionIndex);
	}
	public MGraphNode getNodeByMotion(int motionIndex){
		return motionToNode[motionIndex];
	}

	public MGraphEdge[][] getTransposedEdgeList() {
		return transposedEdgeList;
	}

	public MGraphEdge[] getAdjacentEdgeList() {
		return adjacentEdgeList;
	}
	
	public MGraphEdge[] getTransposedAdjacentEdgeList() {
		return transposedAdjacentEdgeList;
	}

	public MGraphEdge findEdge(int source, int target){
		MGraphEdge[] edges = edgeList[source];
		for (MGraphEdge edge : edges) {
			if (edge.target.index == target){
				return edge;
			}
		}
		return null;
	}

	private MGraphEdge[][] loadEdgeList(){
		try {
			String fileName = Configuration.MGRAPH_EDGE_CACHE_FILE;
			if (new File(fileName).exists() == false){
				throw new RuntimeException();
			}
			
			ObjectInputStream oi = new ObjectInputStream(new BufferedInputStream(new FileInputStream(fileName)));
			// double edgeWeightLimit = oi.readDouble();
			oi.readDouble();
			
			int n = oi.readInt();
			int edgeCount = 0;
			MGraphEdge[][] edgeList = new MGraphEdge[n][];
			for (int i = 0; i < n; i++) {
				int edgeLen = oi.readInt();
				edgeList[i] = new MGraphEdge[edgeLen];
				for (int j = 0; j < edgeLen; j++) {
					int sIndex = oi.readInt();
					int tIndex = oi.readInt();
					double weight = oi.readDouble();
					double kineticEnergy = oi.readDouble();
					int transition = oi.readInt();
					
					double rotY = oi.readDouble();
					double transX = oi.readDouble();
					double transZ = oi.readDouble();
					
					MGraphEdge edge = new MGraphEdge(nodeList[sIndex], nodeList[tIndex], transition, weight, kineticEnergy);
					Matrix4d transform = new Matrix4d();
					transform.rotY(rotY);
					transform.setTranslation(new Vector3d(transX, 0, transZ));
					edge.transform = transform;
					
					edgeList[i][j] = edge;
					edgeCount++;
				}
			}
			System.out.println("Edge count : " + edgeCount);
			oi.close();
			
			return edgeList;
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
	}
	
	
	private MGraphNode[] loadNodeList(){
		try {
			String fileName = Configuration.MGRAPH_NODE_CACHE_FILE;
			if (new File(fileName).exists() == false){
				throw new RuntimeException();
			}
			
			Motion[] motionList = database.getMotionList();
			motionToNode = new MGraphNode[motionList.length];
			
			ObjectInputStream oi = new ObjectInputStream(new BufferedInputStream(new FileInputStream(fileName)));
			int remainSize = oi.readInt();
			MGraphNode[] nodeList = new MGraphNode[remainSize];
			
			for (int i = 0; i < nodeList.length; i++) {
				int index = oi.readInt();
				nodeList[i] = new MGraphNode(this, i, motionList[index]);
				motionToNode[index] = nodeList[i];
			}
			oi.close();
			
			System.out.println("Node count : " + nodeList.length);
			
			return nodeList;
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
	}
	

	public static class MGraphNode{
		public MGraph graph;
		public int index;
		public int motionIndex;
		public Motion motion;
		
		public MGraphNode(MGraph graph, int index, Motion motion) {
			this.graph = graph;
			this.index = index;
			this.motionIndex = motion.motionIndex;
			this.motion = motion;
		}
		
		public int index(){
			return index;
		}
		
		public String filename(){
			return graph.database.getMotionInfoList()[motionIndex].fileName;
		}
		public int frameIndex(){
			return graph.database.getMotionInfoList()[motionIndex].frameIndex;
		}
	}
	
	public static class MGraphEdge{
		public MGraphNode source;
		public MGraphNode target;
		public int transition;
		public double weight;
		public double kineticEnergy;
		
		public Matrix4d transform;
		
		public MGraphEdge(MGraphNode source, MGraphNode target, int transition, double weight,
				double kineticEnergy) {
			this.source = source;
			this.target = target;
			this.transition = transition;
			this.weight = weight;
			this.kineticEnergy = kineticEnergy;
		}
		
		public boolean isTransition(){
			return transition == 1;
		}
		
		public boolean isSequential(){
			return transition == 0;
		}
	}
	
}
