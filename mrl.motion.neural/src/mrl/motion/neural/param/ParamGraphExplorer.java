package mrl.motion.neural.param;

import java.util.ArrayList;
import java.util.Random;

import mrl.motion.neural.param.MotionParamGraph.MPLink;
import mrl.motion.neural.param.MotionParamGraph.MPNode;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class ParamGraphExplorer {
	
	public static int TRANSITION_LIMIT = 30;
	public static int CHECK_MARGIN = 60;

	private MPNode[] nodeList;
	public int[] visitCounts;
	private ArrayList<ArrayList<MPLink>> transitionMap;
	private MPLink[] sequentialLinks;
	
	private Random rand = MathUtil.random;
	
	public ArrayList<Integer> interactionFrames;
	public static int MAX_NON_INTERACTION_LIMIT = 30*4;

	public ParamGraphExplorer(MotionParamGraph graph) {
		nodeList = Utils.toArray(graph.nodeList());
		for (int i = 0; i < nodeList.length; i++) {
			if (nodeList[i].index() != i) throw new RuntimeException(Utils.toString(i, nodeList[i].index()));
		}
		visitCounts = new int[nodeList.length];
		MPLink[][] linkMap = graph.getLinksBySource();
		sequentialLinks = new MPLink[nodeList.length];
		transitionMap = new ArrayList<ArrayList<MPLink>>();
		for (int i = 0; i < sequentialLinks.length; i++) {
			ArrayList<MPLink> tList = new ArrayList<MPLink>();
			for (MPLink link : linkMap[i]){
				if (link.isSequential){
					sequentialLinks[i] = link;
					if (link.source().index() +1 != link.target().index()){
						throw new RuntimeException(Utils.toString(link.source().index(), link.target().index(), link.source().motionIndex, link.target().motionIndex));
					}
				} else {
					tList.add(link);
				}
			}
			transitionMap.add(tList);
		}
	}
	
	public ArrayList<int[]> explore(int size){
		MPNode current = nodeList[rand.nextInt(nodeList.length)];
		ArrayList<int[]> segmentList = new ArrayList<int[]>();
		int totalLen = 0;
		
		int prevStart = current.motionIndex;
		int nonInteractionLen = 0;
		while (true){
			ArrayList<Candidate> candidates = getCandidates(current);
			Candidate c = pick(candidates, nonInteractionLen);
			int startMIdx = current.motionIndex;
			while (true){
				visitCounts[current.index()]++;
				if (current == c.link.source()) break;
				if (sequentialLinks[current.index()] == null){
					System.out.println("ERRR :: " + prevStart + ":: " + startMIdx + " : " + current.motionIndex + " :: " + c.link.source().motionIndex +" // " + c.link.target().motionIndex + " : " + c.link.isSequential);
				}
				current = sequentialLinks[current.index()].target();
				nonInteractionLen++;
			}
			if (!c.link.isSequential){
				segmentList.add(new int[]{ prevStart, current.motionIndex });
				totalLen += (current.motionIndex - prevStart) + 1;
				if (totalLen >= size) break;
				prevStart = c.link.target().motionIndex;
				nonInteractionLen++;
			}
			current = c.link.target();
			if (!c.link.isSequential){
				for (int i = 0; i < TRANSITION_LIMIT; i++) {
					if (sequentialLinks[current.index()] == null) break;
					visitCounts[current.index()]++;
					current = sequentialLinks[current.index()].target();
					nonInteractionLen++;
				}
			}
			if (isInteraction(c.link)){
				nonInteractionLen = 0;
			}
				
		}
		

		int zeroCount = 0;
		for (int c : visitCounts){
			if (c == 0) zeroCount++;
		}
		System.out.println("zero Count :: " + zeroCount + " / " + visitCounts.length + " :: " + zeroCount/(double)visitCounts.length);
		return segmentList;
	}
	
	private Candidate pick(ArrayList<Candidate> candidates, int nonInteractionCount){
		int itcCount1 = 0;
		int itcCount2 = 0;
		boolean isItcOver = (nonInteractionCount > MAX_NON_INTERACTION_LIMIT);
		for (Candidate candidate : candidates){
			MPNode target = candidate.link.target();
			ArrayList<Candidate> cc = getCandidates(target);
			int minCount = getMinimumVCount(target.index());
			for (Candidate c : cc){
				minCount = Math.min(minCount, getMinimumVCount(c.link.target().index())); 
			}
			candidate.vCount = minCount;
			if (nonInteractionCount > MAX_NON_INTERACTION_LIMIT){
				double offset = 1000000;
				if (isInteraction(candidate.link)){
					candidate.offset = offset;
					itcCount1++;
//					System.out.println("isInteraction");
				} else if (isInteractionable(target.index())){
					candidate.offset = offset/1000;
					itcCount2++;
//					System.out.println("isInteractionable");
				}
			}
		}
		double pSum = 0;
		double[] accList = new double[candidates.size()];
		for (int i = 0; i < candidates.size(); i++) {
			double p = 1d/(candidates.get(i).vCount + 0.5);
			p = p*p;
			p += candidates.get(i).offset*10;
			pSum += p;
			accList[i] = pSum;
		}
		double p = rand.nextDouble()*pSum;
		for (int i = 0; i < accList.length; i++) {
			if (p <= accList[i]){
				return candidates.get(i);
			}
		}
		throw new RuntimeException();
	}
	
	private boolean isInteraction(MPLink link){
		if (interactionFrames == null) return false;
		for (int index : interactionFrames){
			int target = link.target().motionIndex;
			if (target < index && target >= index - TRANSITION_LIMIT*3){
				return true;
			}
		}
		return false;
	}
	
	private ArrayList<Candidate> getCandidates(MPNode node){
		ArrayList<Candidate> candidates = new ArrayList<Candidate>();
		for (int i = 0; i < CHECK_MARGIN; i++) {
			int idx = node.index()+i;
			if (idx >= nodeList.length) return candidates;
			for (MPLink link : transitionMap.get(idx)){
				if (link.source().index() != idx){
					throw new RuntimeException(Utils.toString(link.source().index(), idx, node.index()));
				}
				candidates.add(new Candidate(link));
			}
			if (sequentialLinks[idx] == null){
				return candidates;
			}
		}
		int last = node.index() + CHECK_MARGIN - 1;
		if (last < nodeList.length && sequentialLinks[last] != null){
			candidates.add(new Candidate(sequentialLinks[last]));
		}
		return candidates;
	}
	
	private int getMinimumVCount(int index){
		int minCount = visitCounts[index];
		for (int i = 0; i < CHECK_MARGIN; i++) {
			int idx = index+i;
			if (idx >= nodeList.length) break;
			minCount = Math.min(minCount, visitCounts[idx]);
			if (i >= TRANSITION_LIMIT){
				for (MPLink link : transitionMap.get(idx)){
					minCount = Math.min(minCount, visitCounts[link.target().index()]);
				}
			}
			if (sequentialLinks[idx] == null) break;
		}
		return minCount;
	}
	
	private boolean isInteractionable(int index){
		if (interactionFrames == null) return false;
		for (int i = 0; i < CHECK_MARGIN; i++) {
			int idx = index+i;
			if (idx >= nodeList.length) break;
			if (interactionFrames.contains(nodeList[i].motionIndex)) return true;
			if (i >= TRANSITION_LIMIT){
				for (MPLink link : transitionMap.get(idx)){
					if (isInteraction(link)) return true;
				}
			}
			if (sequentialLinks[idx] == null) break;
		}
		return false;
	}
	
	
	private static class Candidate{
		MPLink link;
		double vCount;
		double offset = 0;
		
		public Candidate(MPLink link) {
			this.link = link;
		}
	}
}
