package mrl.motion.neural.basket;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import javax.vecmath.Point2d;

import mrl.motion.data.clip.MClip;
import mrl.motion.data.clip.MClipManager;
import mrl.motion.neural.basket.BasketGraph.BasketNode;
import mrl.util.MathUtil;
import mrl.util.Pair;
import mrl.util.Utils;

public class BasketSequenceGenerator {
	
	public static double MAX_DISTANCE = 6;

	private MClipManager cManager;
	private ConnectableClipMap[] cMapList;
	private BasketGraph graph;
	
	public boolean dribbleOnly = false;

	public BasketSequenceGenerator(MClipManager cManager){
		this.cManager = cManager;
		cMapList = new ConnectableClipMap[cManager.totalClips.size()];
		for (int i = 0; i < cMapList.length; i++) {
			cMapList[i] = new ConnectableClipMap();
		}
		graph = new BasketGraph();
	}
	
	private class History{
		int maxSize = 100;
		LinkedList<BasketNode> nodeHistory = new LinkedList<BasketNode>();
		LinkedList<MClip> clipHistory = new LinkedList<MClip>();
		public void add(BasketNode node, MClip clip){
			nodeHistory.add(node);
			clipHistory.add(clip);
			if (nodeHistory.size() > maxSize){
				nodeHistory.removeFirst();
				clipHistory.removeFirst();
			}
		}
		
		public Pair<BasketNode, MClip> reverse(){
			nodeHistory.removeLast();
			clipHistory.removeLast();
			nodeHistory.removeLast();
			clipHistory.removeLast();
			
			return new Pair<BasketNode, MClip>(nodeHistory.getLast(), clipHistory.getLast());
		}
	}
	
	private ArrayList<MClip> candidates(MClip clip, String label){
		ArrayList<MClip> candidates = cMapList[clip.index].get(label);
		if (candidates == null){
			candidates = getCandidates(clip, label);
			cMapList[clip.index].put(label, candidates);
		}
		return candidates;
	}
	
	public ArrayList<MClip> generate(int size){
		BasketNode current = graph.getNode("shoot");
		if (current == null){
			current = graph.getNode("dribble");
		}
//		BasketNode current = graph.nodeList.get(0);
		if (dribbleOnly){
			current = graph.getNode("dribble");
		}
		MClip prevClip = null;
		BasketNode prevNode = null;
		ArrayList<MClip> result = new ArrayList<MClip>();
		int retryCount = 0;
		int sucCount = 0;
		History h = new History();
		h.add(current, null);
		for (int i = 0; i < size; i++) {
			ArrayList<MClip> candidates;
			if (prevClip == null){
				candidates = cManager.labelMap.get(current.label);
			} else {
				candidates = candidates(prevClip, current.label);
			}
			if (candidates.size() == 0){
				System.out.println("removed " + result.remove(result.size()-1));
				System.out.println("removed " + result.remove(result.size()-1));
				Pair<BasketNode, MClip> pair = h.reverse();
				System.out.println("reverse:: " + prevClip.type + " -> " + current.label + " :: " + pair.first.label + " : " + pair.second);
//				System.out.println("reverse:: " + current.label + "->" + pair.first.label + " : " + pair.second);
				current = pair.first;
				prevClip = pair.second;
				i-=3;
				retryCount++;
				if (retryCount > 15){
					System.out.println("no candidates :: " + prevClip.type + " :: " + current.label);
					throw new RuntimeException();
				}
				sucCount = 0;
				continue;
			}
			
//			if (candidates.size() == 0){
//				System.out.println("pre revserce :: " + prevNode.label + " -> "
//								+ current.label + " : " + prevClip.type);
//				current = prevNode;
//				MClip removed = result.remove(result.size()-1);
//				prevClip = result.size() > 0 ? Utils.last(result) : null;
//				i--;
//				retryCount++;
//				if (retryCount > 15){
//					System.out.println("no candidates :: " + prevClip.type + " :: " + current.label);
//					throw new RuntimeException();
//				}
//				try{
//					System.out.println("reverse :: " + i + " : " + prevNode.label + " -> "
//							+ current.label + " : " + prevClip.type + " , " + removed.type);
//				} catch (NullPointerException e){
//				}
//				sucCount = 0;
//				continue;
//			}
			
			sucCount++;
			if (sucCount > 2){
				retryCount = 0;
			}
			
			prevNode = current;
			current = current.navigateNext();
			MClip nextClip = pick(candidates, prevClip, current.label);
//			MClip nextClip = Utils.pickRandom(candidates, MathUtil.random);
			result.add(nextClip);
			h.add(current, nextClip);
			if (dribbleOnly){
				current = graph.getNode("dribble");
			}
			System.out.println(result.size() + " : " +  i + " : " + prevNode.label + " -> " + current.label + " : " + nextClip);
			
			prevClip = nextClip;
		}
		return result;
	}
	
	
	public static boolean PICK_RANDOM = false;
	private MClip pick(ArrayList<MClip> candidates, MClip prevClip, String nextLabel){
		ArrayList<MClip> available = new ArrayList<MClip>();
		for (MClip c : candidates){
			if (candidates(c, nextLabel).size() > 0){
				available.add(c);
			}
		
		}
		// no way
		if (available.size() == 0) return candidates.get(0);
		
		candidates = available;
		if (prevClip == null || PICK_RANDOM){
			return Utils.pickRandom(candidates, MathUtil.random);
		}
		
		double pSum = 0;
		double[] accList = new double[candidates.size()];
		for (int i = 0; i < candidates.size(); i++) {
			Point2d p1 = prevClip.transform.position;
			Point2d p2 = prevClip.transform.localToGlobal(candidates.get(i).transform.position);
			double diff = MathUtil.length(p2) - MathUtil.length(p1);
			diff /= candidates.get(i).length();
			diff = Math.min(2.5, diff);
			diff = Math.max(0.1, diff);
			
			double p = diff*diff;
			pSum += p;
			accList[i] = pSum;
		}
		double p = MathUtil.random.nextDouble()*pSum;
		for (int i = 0; i < accList.length; i++) {
			if (p <= accList[i]){
				return candidates.get(i);
			}
		}
		throw new RuntimeException();
	}
	
	private ArrayList<MClip> getCandidates(MClip prevClip, String label){
		ArrayList<MClip> connectableList = new ArrayList<MClip>();
		ArrayList<MClip> clipList = cManager.labelMap.get(label);
		for (MClip c : clipList){
			if (c.toString().equals("feint_l(s_006_7_1:166:253)")) continue;
			if (cManager.distanceMap[prevClip.index][c.index].distance < MAX_DISTANCE){
				connectableList.add(c);
			}
		}
		return connectableList;
		
	}
	
	private static class ConnectableClipMap extends HashMap<String, ArrayList<MClip>>{
		private static final long serialVersionUID = -1778211418471485016L;
	}
}
