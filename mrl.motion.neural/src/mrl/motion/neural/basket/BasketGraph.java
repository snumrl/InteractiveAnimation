package mrl.motion.neural.basket;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import mrl.motion.neural.data.BasketPickupControl;
import mrl.util.MathUtil;
import mrl.util.Pair;

public class BasketGraph {
	
	public static boolean PASS_MODE = false;
	public static boolean PICKUP_MODE = false;
	public static boolean INCLUDE_CATCH = false;
	public static boolean ALL_TIME = false;
	
	private static Random rand = MathUtil.random;

	public ArrayList<BasketNode> nodeList = new ArrayList<BasketNode>();
	private ArrayList<BasketGroup> groupList = new ArrayList<BasketGroup>();
	private ArrayList<BasketLink> linkList = new ArrayList<BasketGraph.BasketLink>();
	private HashMap<String, BasketGroup> groupMap = new HashMap<String, BasketGroup>();
	private HashMap<Pair<BasketNode, BasketNode>, BasketLink> linkMap = new HashMap<Pair<BasketNode,BasketNode>, BasketLink>();
	
	public BasketGraph(){
		if (PICKUP_MODE){
			pickupGraph();
		} else if (PASS_MODE){
			passGraph();
		} else if (ALL_TIME){
			allTimeGraph();
		} else {
			gameGraph();
		}
	}
	
	public void pickupGraph(){
		BasketGroup Move = makeGroup("Move", "run", "walk", "idle");
		BasketGroup Pickup;
		if (INCLUDE_CATCH){
			Pickup = makeGroup("Pickup", "pickup", "catch");
		} else {
			Pickup = makeGroup("Pickup", "pickup");
		}
//		BasketGroup Pickup = makeGroup("Pickup", "pickup");
		BasketGroup Pivot1 = makePivot("Pivot1");
		BasketGroup Dribble = makeGroup("Dribble", "dribble");
		BasketGroup Pivot2 = makePivot("Pivot2");
		BasketGroup Shoot = makeGroup("Shoot", "shoot", "shoot_near");
		
		addLink(Move, Move);
		addLinks(Move, Pickup, Pivot1, Dribble, Pivot2, Shoot);
		addLink(Pickup, Dribble);
		addLink(Dribble, Dribble);
		addLink(Dribble, Shoot);
		addLink(Shoot, Move);
		
		linksByTarget("fake_shoot").forEach(l -> l.weight = 0);
		
		linksByTarget("walk").forEach(l -> l.weight = 3.5);
		linksByTarget("dribble").forEach(l -> l.weight = 3);
		links("run", "run").forEach(l -> l.weight = 2);
		links("run", "walk").forEach(l -> l.weight = 2.5);
		links("walk", "walk").forEach(l -> l.weight = 4);
		links("dribble", "dribble").forEach(l -> l.weight = 5);
		linksByTarget("idle").forEach(l -> l.weight = 0.5);
		
		linksByTarget("pivot_l").forEach(l -> l.weight = 0.2);
		linksByTarget("pivot_r").forEach(l -> l.weight = 0.2);
		links("pivot_l", "pivot_l").forEach(l -> l.weight = 0.5);
		links("pivot_r", "pivot_r").forEach(l -> l.weight = 0.5);
		
		linksByTarget("shoot_near").forEach(l -> l.weight = 0);
		links("dribble", "shoot_near").forEach(l -> l.weight = 1.5);
		
		if (INCLUDE_CATCH){
			links("idle", "catch").forEach(l -> l.weight = 0);
		}
	}
	
	public void passGraph(){
		BasketGroup Move = makeGroup("Move", "run", "walk", "idle");
		BasketGroup Receive = makeGroup("Receive", "pass'");
//		BasketGroup Pickup = makeGroup("Pickup", "pickup");
		BasketGroup Dribble = makeGroup("Dribble", "dribble");
		BasketGroup Pass = makeGroup("Pass", "pass");
		
		addLink(Move, Move);
		addLinks(Move, Receive, Dribble, Pass);
		addLink(Dribble, Dribble);
		addLink(Pass, Move);
		
		linksByTarget("walk").forEach(l -> l.weight = 1.5);
		linksByTarget("pass'").forEach(l -> l.weight = 6);
		linksByTarget("dribble").forEach(l -> l.weight = 3);
		links("dribble", "dribble").forEach(l -> l.weight = 3);
		links("run", "run").forEach(l -> l.weight = 2);
		links("run", "walk").forEach(l -> l.weight = 1.5);
		links("walk", "walk").forEach(l -> l.weight = 2);
		linksByTarget("idle").forEach(l -> l.weight = 0.5);
	}
	
	public void allTimeGraph(){
		BasketGroup Move = makeGroup("Move", "run", "walk", "idle");
//		STGGroup Move = makeGroup("Move", "run", "walk", "idle", "screen");
		
		BasketGroup Catch = makeGroup("Catch", "feint_l", "feint_r", "run", "pass'");
		Catch.setHeads("feint_l", "feint_r", "pass'");
		Catch.setTails("pass'");
		Catch.addLink("feint_l", "run");
		Catch.addLink("feint_r", "run");
		Catch.addLink("feint_l", "pass'");
		Catch.addLink("feint_r", "pass'");
		Catch.addLink("run", "run");
		Catch.addLink("run", "pass'");
		
		BasketGroup Pivot1 = makePivot("Pivot1");
		BasketGroup Dribble = makeGroup("Dribble", "dribble");
		BasketGroup Pivot2 = makePivot("Pivot2");
		BasketGroup Pass = makeGroup("Pass", "pass", "shoot", "shoot_near");
//		BasketGroup Shoot = makeGroup("Shoot", "shoot", "shoot_near");
		
		addLink(Move, Move);
		addLinks(Move, Catch, Pivot1, Dribble, Pivot2, Pass);
		addLink(Catch, Dribble);
		addLink(Catch, Pivot2);
		addLink(Catch, Pass);
		addLink(Pivot1, Pass);
		addLink(Dribble, Pass);
		addLink(Dribble, Dribble);
		
//		addLink(Catch, Shoot);
//		addLink(Pivot1, Shoot);
//		addLink(Dribble, Shoot);
		
		addLink(Pass, Move);
		
		
		links("pass'", "pass").forEach(l -> l.weight = 0.15);
		links("pass'", "shoot").forEach(l -> l.weight = 0.25);
		linksByTarget("dribble").forEach(l -> l.weight = 4);
		links("pass'", "dribble").forEach(l -> l.weight = 16);
		links("dribble", "dribble").forEach(l -> l.weight = 8);
//		linksByTarget("pass_long").forEach(l -> l.weight = 0.2);
		
		for (BasketNode node1 : Move.nodeMap.values()){
			for (BasketNode node2 : Move.nodeMap.values()){
				double weight = 2;
				if (node1 == node2) weight = 3;
				linkMap.get(new Pair<>(node1, node2)).weight = weight;
			}
		}
		
		linksByTarget("pivot_l").forEach(l -> l.weight = 0.25);
		linksByTarget("pivot_r").forEach(l -> l.weight = 0.25);
		links("pivot_l", "pivot_l").forEach(l -> l.weight = 0.5);
		links("pivot_r", "pivot_r").forEach(l -> l.weight = 0.5);
		
		linksByTarget("walk").forEach(l -> l.weight = 2);
		linksByTarget("idle").forEach(l -> l.weight = 0.25);
		linksByTarget("run").forEach(l -> l.weight = 1.5);
		links("walk", "walk").forEach(l -> l.weight = 3);
		links("run", "run").forEach(l -> l.weight = 2);
		links("run", "idle").forEach(l -> l.weight = 0.1);
		links("idle", "idle").forEach(l -> l.weight = 1);
		links(Move, Catch).forEach(l -> l.weight = 2);
		
		linksByTarget("feint").forEach(l -> l.weight = 0.05);
		
		linksByTarget("shoot_near").forEach(l -> l.weight = 0);
		links("dribble", "shoot_near").forEach(l -> l.weight = 1);
		links("dribble", "pass").forEach(l -> l.weight = 2.5);
		
		
//		linksByTarget("pass'").forEach(l -> System.out.println(l));
//		linksByTarget("dribble").forEach(l -> linksBySource(l.source.label).forEach(l2 -> System.out.println(l2)));
//		links("dribble", "dribble").forEach(l -> l.weight = 5);
		
		System.out.println("link size :: " + linkList.size());
	}
	
	
	public void gameGraph(){
		BasketGroup Move = makeGroup("Move", "run", "walk", "idle");
//		STGGroup Move = makeGroup("Move", "run", "walk", "idle", "screen");
		
		BasketGroup Catch;
		
		if (INCLUDE_CATCH){
			Catch = makeGroup("Catch", "feint_l", "feint_r", "run", "pass'", "catch");
			Catch.setHeads("feint_l", "feint_r", "pass'", "catch");
			Catch.setTails("pass'", "catch");
		} else {
			Catch = makeGroup("Catch", "feint_l", "feint_r", "run", "pass'");
			Catch.setHeads("feint_l", "feint_r", "pass'");
			Catch.setTails("pass'");
		}
		Catch.addLink("feint_l", "run");
		Catch.addLink("feint_r", "run");
		Catch.addLink("feint_l", "pass'");
		Catch.addLink("feint_r", "pass'");
		Catch.addLink("run", "run");
		Catch.addLink("run", "pass'");
		if (INCLUDE_CATCH){
			Catch.addLink("feint_l", "catch");
			Catch.addLink("feint_r", "catch");
			Catch.addLink("run", "catch");
		}
		
		BasketGroup Pivot1 = makePivot("Pivot1");
		BasketGroup Dribble = makeGroup("Dribble", "dribble");
		BasketGroup Pivot2 = makePivot("Pivot2");
		BasketGroup Pass = makeGroup("Pass", "pass", "shoot", "shoot_near");
//		BasketGroup Shoot = makeGroup("Shoot", "shoot", "shoot_near");
		
		addLink(Move, Move);
		addLinks(Move, Catch, Pivot1, Dribble, Pivot2, Pass);
		addLink(Catch, Dribble);
		addLink(Catch, Pivot2);
		addLink(Catch, Pass);
		addLink(Pivot1, Pass);
		addLink(Dribble, Pass);
		addLink(Dribble, Dribble);
		
//		addLink(Catch, Shoot);
//		addLink(Pivot1, Shoot);
//		addLink(Dribble, Shoot);
		
		addLink(Pass, Move);
		
		
		links("pass'", "pass").forEach(l -> l.weight = 0.25);
		links("pass'", "shoot").forEach(l -> l.weight = 0.25);
		linksByTarget("dribble").forEach(l -> l.weight = 3);
		links("pass'", "dribble").forEach(l -> l.weight = 10);
		links("dribble", "dribble").forEach(l -> l.weight = 6);
		
		if (INCLUDE_CATCH){
			links("catch", "pass").forEach(l -> l.weight = 0.25);
			links("catch", "shoot").forEach(l -> l.weight = 0.25);
			links("catch", "dribble").forEach(l -> l.weight = 10);
		}
		
		for (BasketNode node1 : Move.nodeMap.values()){
			for (BasketNode node2 : Move.nodeMap.values()){
				double weight = 2;
				if (node1 == node2) weight = 3;
				linkMap.get(new Pair<>(node1, node2)).weight = weight;
			}
		}
		
		linksByTarget("pivot_l").forEach(l -> l.weight = 0.3);
		linksByTarget("pivot_r").forEach(l -> l.weight = 0.3);
		links("pivot_l", "pivot_l").forEach(l -> l.weight = 0.5);
		links("pivot_r", "pivot_r").forEach(l -> l.weight = 0.5);
		
		linksByTarget("walk").forEach(l -> l.weight = 3);
		linksByTarget("idle").forEach(l -> l.weight = 0.25);
		links("walk", "walk").forEach(l -> l.weight = 6);
		links("run", "run").forEach(l -> l.weight = 2);
		links("run", "idle").forEach(l -> l.weight = 0.1);
		links("idle", "idle").forEach(l -> l.weight = 1);
		links(Move, Catch).forEach(l -> l.weight = 2);
		
		linksByTarget("feint").forEach(l -> l.weight = 0.05);
		
		linksByTarget("shoot_near").forEach(l -> l.weight = 0);
		links("dribble", "shoot_near").forEach(l -> l.weight = 1.5);
		links("dribble", "pass").forEach(l -> l.weight = 2);
		if (INCLUDE_CATCH){
			links("idle", "catch").forEach(l -> l.weight = 0);
		}
		
//		linksByTarget("pass'").forEach(l -> System.out.println(l));
//		linksByTarget("dribble").forEach(l -> linksBySource(l.source.label).forEach(l2 -> System.out.println(l2)));
//		links("dribble", "dribble").forEach(l -> l.weight = 5);
		
		System.out.println("link size :: " + linkList.size());
	}
	
//	public void gameGraph(){
//		BasketGroup Move = makeGroup("Move", "run", "walk", "idle");
////		STGGroup Move = makeGroup("Move", "run", "walk", "idle", "screen");
//		
//		BasketGroup Catch = makeGroup("Catch", "feint_l", "feint_r", "run", "pass'");
//		Catch.setHeads("feint_l", "feint_r", "pass'");
//		Catch.setTails("pass'");
//		Catch.addLink("feint_l", "run");
//		Catch.addLink("feint_r", "run");
//		Catch.addLink("feint_l", "pass'");
//		Catch.addLink("feint_r", "pass'");
//		Catch.addLink("run", "run");
//		Catch.addLink("run", "pass'");
//		
//		BasketGroup Pivot1 = makePivot("Pivot1");
//		BasketGroup Dribble = makeGroup("Dribble", "dribble");
//		BasketGroup Pivot2 = makePivot("Pivot2");
//		BasketGroup Pass = makeGroup("Pass", "pass", "pass_long", "shoot");
////		BasketGroup Shoot = makeGroup("Shoot", "shoot", "shoot_near");
//		
//		addLink(Move, Move);
//		addLinks(Move, Catch, Pivot1, Dribble, Pivot2, Pass);
//		addLink(Catch, Dribble);
//		addLink(Catch, Pivot2);
//		addLink(Catch, Pass);
//		addLink(Pivot1, Pass);
//		addLink(Dribble, Pass);
//		addLink(Dribble, Dribble);
//		
////		addLink(Catch, Shoot);
////		addLink(Pivot1, Shoot);
////		addLink(Dribble, Shoot);
//		
//		addLink(Pass, Move);
//		
//		
//		links("pass'", "pass").forEach(l -> l.weight = 0.25);
//		links("pass'", "shoot").forEach(l -> l.weight = 0.25);
//		links("pass'", "dribble").forEach(l -> l.weight = 3);
//		linksByTarget("dribble").forEach(l -> l.weight = 5);
//		links("dribble", "dribble").forEach(l -> l.weight = 5);
//		linksByTarget("pass_long").forEach(l -> l.weight = 0.2);
//		
//		for (BasketNode node1 : Move.nodeMap.values()){
//			for (BasketNode node2 : Move.nodeMap.values()){
//				double weight = 2;
//				if (node1 == node2) weight = 3;
//				linkMap.get(new Pair<>(node1, node2)).weight = weight;
//			}
//		}
//		
//		linksByTarget("pivot_l").forEach(l -> l.weight = 0.2);
//		linksByTarget("pivot_r").forEach(l -> l.weight = 0.2);
//		links("pivot_l", "pivot_l").forEach(l -> l.weight = 0.5);
//		links("pivot_r", "pivot_r").forEach(l -> l.weight = 0.5);
//		
//		linksByTarget("walk").forEach(l -> l.weight = 5);
//		linksByTarget("idle").forEach(l -> l.weight = 0.25);
//		links("walk", "walk").forEach(l -> l.weight = 2.5);
//		links("run", "run").forEach(l -> l.weight = 2);
//		links("run", "idle").forEach(l -> l.weight = 0.1);
//		links("idle", "idle").forEach(l -> l.weight = 1);
//		links(Move, Catch).forEach(l -> l.weight = 2);
//		
//		linksByTarget("feint").forEach(l -> l.weight = 0.05);
//		
//		
////		linksByTarget("pass'").forEach(l -> System.out.println(l));
////		linksByTarget("dribble").forEach(l -> linksBySource(l.source.label).forEach(l2 -> System.out.println(l2)));
////		links("dribble", "dribble").forEach(l -> l.weight = 5);
//		
//		System.out.println("link size :: " + linkList.size());
//	}
	
	public BasketNode getNode(String label){
		for (BasketNode node : nodeList){
			if (node.label.equals(label)) return node;
		}
		return null;
	}
	
	private ArrayList<BasketLink> linksBySource(String prefix){
		ArrayList<BasketLink> list = new ArrayList<BasketLink>();
		for (BasketLink link : linkList){
			if (link.source.label.startsWith(prefix)){
				list.add(link);
			}
		}
		return list;
	}
	
	private ArrayList<BasketLink> linksByTarget(String prefix){
		ArrayList<BasketLink> list = new ArrayList<BasketLink>();
		for (BasketLink link : linkList){
			if (link.target.label.startsWith(prefix)){
				list.add(link);
			}
		}
		return list;
	}
	
	private ArrayList<BasketLink> links(String source, String target){
		ArrayList<BasketLink> list = new ArrayList<BasketLink>();
		for (BasketLink link : linkList){
			if (link.source.label.equals(source) && link.target.label.equals(target)){
				list.add(link);
			}
		}
		if (list.size() == 0) throw new RuntimeException();
		return list;
	}
	
	private ArrayList<BasketLink> links(BasketGroup source, BasketGroup target){
		ArrayList<BasketLink> list = new ArrayList<BasketLink>();
		for (BasketLink link : linkList){
			if (link.source.group == source && link.target.group == target){
				list.add(link);
			}
		}
		if (list.size() == 0) throw new RuntimeException();
		return list;
	}
	
	private BasketGroup makePivot(String label){
		BasketGroup Pivot = makeGroup(label, "pivot_l", "pivot_r", "fake_shoot");
		Pivot.setHeads("pivot_l", "pivot_r");
		Pivot.setTails("pivot_l", "pivot_r", "fake_shoot");
		Pivot.addLink("pivot_l", "pivot_l");
		Pivot.addLink("pivot_l", "fake_shoot");
		Pivot.addLink("pivot_r", "pivot_r");
		Pivot.addLink("pivot_r", "fake_shoot");
		return Pivot;
	}
	
	private void addLinks(BasketGroup... groups){
		for (int i = 0; i < groups.length-1; i++) {
			addLink(groups[i], groups[i+1]);
		}
	}
	
	private void addLink(BasketGroup source, BasketGroup target){
		for (BasketNode sourceNode : source.tailNodes){
			for (BasketNode targetNode : target.headNodes){
				addLink(sourceNode, targetNode);
			}
		}
	}
	
	private BasketLink addLink(BasketNode source, BasketNode target){
		BasketLink link = new BasketLink(source, target);
		Pair<BasketNode, BasketNode> key = new Pair<BasketNode, BasketNode>(source, target);
		if (linkMap.containsKey(key)){
			throw new RuntimeException();
		}
		source.linkList.add(link);
		linkList.add(link);
		linkMap.put(key, link);
		return link;
	}
	
	private BasketGroup makeGroup(String label, String... types){
		BasketGroup group = new BasketGroup(label);
		for (String type : types) {
			group.addNode(type);
		}
		group.setHeads(types);
		group.setTails(types);
		return group;
	}
	
	public class BasketGroup{
		public String label;
		public HashMap<String, BasketNode> nodeMap = new HashMap<String, BasketNode>();
		
		public ArrayList<BasketNode> headNodes = new ArrayList<BasketNode>();
		public ArrayList<BasketNode> tailNodes = new ArrayList<BasketNode>();
		
		public BasketGroup(String label) {
			this.label = label;
		}
		
		public void addNode(String type){
			BasketNode node = new BasketNode(this, type);
			nodeMap.put(type, node);
		}
		
		public BasketNode get(String type){
			return nodeMap.get(type);
		}
		
		public void setHeads(String... types){
			headNodes.clear();
			for (String type : types){
				headNodes.add(get(type));
			}
		}
		public void setTails(String... types){
			tailNodes.clear();
			for (String type : types){
				tailNodes.add(get(type));
			}
		}
		
		public BasketLink addLink(String sourceType, String targetType){
			return BasketGraph.this.addLink(get(sourceType), get(targetType));
		}
	}
	
	public class BasketNode{
		public BasketGroup group;
		public String label;
		public int index;
		public ArrayList<BasketLink> linkList = new ArrayList<BasketLink>();
		
		private double weightSum = -1;
		private double[] weightAcc = null;
		
		
		public BasketNode(BasketGroup group, String label) {
			this.group = group;
			this.label = label;
			this.index = nodeList.size();
			nodeList.add(this);
		}
		
		public BasketNode navigateNext(){
			if (weightAcc == null){
				weightSum = 0;
				weightAcc = new double[linkList.size()];
				for (int i = 0; i < linkList.size(); i++) {
					weightSum += linkList.get(i).weight;
					weightAcc[i] = weightSum;
				}
			}
			double r = rand.nextDouble()*weightSum;
			for (int i = 0; i < weightAcc.length; i++) {
				if (r <= weightAcc[i]) return linkList.get(i).target;
			}
			throw new RuntimeException();
		}
		
		@Override
		public String toString(){
			return "(" + group.label + ")" + label;
		}
	}
	
	public class BasketLink{
		public BasketNode source;
		public BasketNode target;
		public double weight = 1;
		
		public BasketLink(BasketNode source, BasketNode target) {
			this.source = source;
			this.target = target;
		}
		
		public String toString(){
			return source.toString() + " -> " + target.toString();
		}
		
	}
	
	public static void main(String[] args) {
		BasketGraph g = new BasketGraph();
		System.out.println("#####################");
		BasketNode current = g.nodeList.get(0);
		for (int i = 0; i < 100; i++) {
			System.out.println(current);
			current = current.navigateNext();
		}
		
	}
}
