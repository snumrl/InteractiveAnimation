package mrl.motion.neural.param;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

import mrl.graph.DefaultGraph;
import mrl.graph.DefaultLink;
import mrl.graph.DefaultNode;
import mrl.graph.StrongComponent;
import mrl.motion.data.Contact;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.trasf.LazyDist;
import mrl.util.FileUtil;
import mrl.util.Utils;

public class MotionParamGraph extends DefaultGraph<MotionParamGraph.MPNode, MotionParamGraph.MPLink> {

	public static double DIST_LIMIT = 80;
	private static int BALL_MIN_MARGIN = 6;
	public MDatabase database;
	public Motion[] mList;
//	public double[][] distMap;
	
	private LazyDist lazyDist;
	private int[] prevBallMargin;
	private int[] postBallMargin;
	private int[] ballLeftOffset;
	
	public HashSet<Integer> removedSet;

	public MotionParamGraph(MDatabase database) {
		this.database = database;
		
		mList = database.getMotionList();
		prevBallMargin = new int[mList.length];
		postBallMargin = new int[mList.length];
		ballLeftOffset = new int[mList.length];
		for (int i = 0; i < prevBallMargin.length; i++) {
			Contact contact = mList[i].ballContact;
			int margin = 0;
			for (int m = 1; m <= BALL_MIN_MARGIN; m++) {
				int idx = i - m;
				if (idx < 0) break;
				if (contact.equals(mList[idx].ballContact)){
					margin++;
				}
			}
			prevBallMargin[i] = margin;
			
			margin = 0;
			for (int m = 1; m <= BALL_MIN_MARGIN; m++) {
				int idx = i + m;
				if (idx >= mList.length) break;
				if (contact.equals(mList[idx].ballContact)){
					margin++;
				}
			}
			postBallMargin[i] = margin;
			
			ballLeftOffset[i] = ballLeftOffset(i, 10);
		}
		
		lazyDist = new LazyDist(database.getDist(), 2);
		
	}
	
	private int ballLeftOffset(int index, int margin){
		if (mList[index].ballContact.isNoContact()) return margin;
		for (int i = 1; i < margin; i++) {
			int idx = index + i;
			if (idx < 0 || idx >= mList.length) continue;
			if (mList[idx].ballContact.isNoContact()) return i;
		}
		return margin;
	}
	
	public void makeGraph(){
		int margin = 10;
		int boundaryMargin = 100;
		
		ArrayList<Integer> validList = new ArrayList<Integer>();
		for (int i = margin; i < mList.length - margin; i++) {
			if (!isInBoundary(mList[i], boundaryMargin)) continue;
			validList.add(i);
		}
		System.out.println("valid len :: " + validList.size());
		
		for (int i = 0; i < validList.size(); i++) {
			Motion m = mList[validList.get(i)];
			addNode(new MPNode(m.motionIndex));
			if (nodeList.size() > 1){
				MPNode prevNode = nodeList().get(i-1);
				Motion prev = mList[prevNode.motionIndex];
				if (prev.next == m){
					addLink(new MPLink(prevNode, nodeList.get(i), true));
				}
			}
		}
		
		
		for (int i = 0; i < nodeList().size(); i++) {
			if (i%50 == 0 ) System.out.println("makeGraph progress :: " + i + " :: " + linkList().size());
			if (!isInBoundary(mList[nodeList().get(i).motionIndex], boundaryMargin)){
				System.out.println("????????????AWfawfawf :: " + i + " : " + mList[nodeList().get(i).motionIndex]);
			}
			for (int j = 0; j < nodeList().size(); j++) {
				if (i == j) continue;
				int m_i = nodeList().get(i).motionIndex;
				int m_j = nodeList().get(j).motionIndex;
				double d = dist(m_i, m_j);
				if (d > DIST_LIMIT) continue;
				if (isMinimum(m_i, m_j, margin)){
					MPLink link = new MPLink(nodeList().get(i), nodeList().get(j), d);
					addLink(link);
				}
			}
		}
		System.out.println("## graph size :: " + nodeList.size() + " , " + linkList.size());
		HashSet<Integer> originSet = new HashSet<Integer>();
		for (MPNode node : nodeList()){
			originSet.add(node.motionIndex);
		}
		
		while (true){
			new StrongComponent().find(this);
			ArrayList<MPLink> allLinks = new ArrayList<MPLink>();
			allLinks.addAll(linkList());
			MPNode[] prevNode = new MPNode[nodeList().size()];
			for (MPLink link : allLinks){
				if (link.source().motionIndex + 1 == link.target().motionIndex){
					prevNode[link.target().index()] = link.source();
				}
			}
			int removedCount = 0;
			boolean isLinkRemoved = false;
			for (MPLink link : allLinks){
				if (!isTransitionable(link, prevNode)){
					removeLink(link);
					isLinkRemoved = true;
					removedCount++;
				}
			}
			System.out.println("isLinkRemoved : " + isLinkRemoved + " : " + removedCount + " / " + allLinks.size());
			if (!isLinkRemoved) break;
		}
		
		for (MPNode node : nodeList()){
			originSet.remove(node.motionIndex);
		}
		removedSet = originSet;
		System.out.println("## after graph size :: " + nodeList.size() + " , " + linkList.size());
	}
	
	private boolean isTransitionable(MPLink link, MPNode[] prevNode){
		if (link.source().motionIndex + 1 == link.target().motionIndex) return true;
		MPNode node = link.source();
		int tLimit = ParamGraphExplorer.TRANSITION_LIMIT;
		for (int i = 0; i < tLimit + 2; i++) {
			node = prevNode[node.index()];
			if (node == null) return false;
		}
		return true;
	}
	
	public double dist(int i, int j){
		Motion m1 = mList[i+1];
		Motion m2 = mList[j];
		if (!m1.ballContact.equals(m2.ballContact)) return Integer.MAX_VALUE;
		if (ballLeftOffset[i] <= 4 && ballLeftOffset[j] >= 8) return Integer.MAX_VALUE;
		if (ballLeftOffset[j] <= 4 && ballLeftOffset[i] >= 8) return Integer.MAX_VALUE;
		if (prevBallMargin[i+1] + postBallMargin[j] < BALL_MIN_MARGIN) return Integer.MAX_VALUE;
		if (m1.isLeftFootContact != m2.isLeftFootContact) return Integer.MAX_VALUE;
		if (m1.isRightFootContact != m2.isRightFootContact) return Integer.MAX_VALUE;
		return lazyDist.getSeqDist(i+1, j);
	}
	
	public ArrayList<MPLink> getNonSequentialLinks(){
		ArrayList<MPLink> list = new ArrayList<MotionParamGraph.MPLink>();
		for (MPLink link : linkList()){
			if (!link.isSequential){
				list.add(link);
			}
		}
		return list;
	}
	
	protected boolean isInBoundary(Motion m, int margin){
		if (m.motionData.file.getName().contains("94_02") && m.frameIndex < 275){
			return false;
		}
		if (m.motionData.file.getName().contains("94_11")){
			if (m.frameIndex < 445 || m.frameIndex > 800) return false;
		}
				
		if (m.frameIndex < margin || m.frameIndex + margin >= m.motionData.motionList.size()) return false;
		return true;
	}
	
	private boolean isMinimum(int x, int y, int margin){
		double min = dist(x, y);
		for (int i = 0; i <= margin; i++) {
			for (int j = 0; j <= margin; j++) {
				if (i == 0 && j == 0) continue;
				if (!isValid(x, y, x + i,y + j, min)) return false;
				if (!isValid(x, y, x - i,y + j, min)) return false;
				if (!isValid(x, y, x + i,y - j, min)) return false;
				if (!isValid(x, y, x - i,y - j, min)) return false;
			}
		}
		return true;
	}
	
	private boolean isValid(int x, int y, int px, int py, double min){
		if (px < 0 || px >= mList.length) return true;
		if (py < 0 || py >= mList.length) return true;
		if (mList[x].motionData != mList[px].motionData) return true;
		if (mList[y].motionData != mList[py].motionData) return true;
		if (lazyDist.getSeqDist(px, py) < min) return false;
//		if (dist(px, py) < min) return false;
		return true;
	}
	
	public void loadGraph(String file){
		try {
			DataInputStream is = FileUtil.dataInputStream(file);
			int nodeSize = is.readInt();
			for (int i = 0; i < nodeSize; i++) {
				int mIndex = is.readInt();
				addNode(new MPNode(mIndex));
			}
			
			int linkSize = is.readInt();
			for (int i = 0; i < linkSize; i++) {
				MPNode source = nodeList().get(is.readInt());
				MPNode target = nodeList().get(is.readInt());
				double distance = is.readDouble();
				boolean isSequential = is.readBoolean();
				if (isSequential){
					addLink(new MPLink(source, target, isSequential));
				} else {
					addLink(new MPLink(source, target, distance));
				}
			}
			is.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void saveGraph(String file){
		try {
			DataOutputStream os = FileUtil.dataOutputStream(file);
			os.writeInt(nodeList().size());
			for (MPNode node : nodeList()) {
				os.writeInt(node.motionIndex);
			}
			os.writeInt(linkList().size());
			for (MPLink link : linkList()){
				os.writeInt(link.source().index());
				os.writeInt(link.target().index());
				os.writeDouble(link.distance());
				os.writeBoolean(link.isSequential);
			}
			os.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public static class MPNode extends DefaultNode{
		public int motionIndex;

		public MPNode(int motionIndex) {
			this.motionIndex = motionIndex;
		}
		
	}
	public static class MPLink extends DefaultLink<MPNode>{
		public boolean isSequential;
		
		public MPLink(MPNode source, MPNode target, double distance) {
			super(source, target);
			this.distance = distance;
			isSequential = false;
		}
		
		public MPLink(MPNode source, MPNode target, boolean isSequential) {
			super(source, target);
			if (!isSequential) throw new RuntimeException();
			this.isSequential = isSequential;
			this.distance = -1;
		}
	}
	
}
