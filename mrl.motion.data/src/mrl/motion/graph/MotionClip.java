package mrl.motion.graph;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Matrix4d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionAnnotation;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.graph.MGraph.MGraphEdge;
import mrl.motion.graph.MGraph.MGraphNode;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class MotionClip {
	
	public static final String STATE_STANDING = "standing";
	public static final String STATE_BENT = "bent";
	public static final String STATE_LYING = "lying";

	public String file;
	public String oppositeFile;
	public int startFrame;
	public int endFrame;
	public int interactionFrame = -1;

	public int startMotionIndex;
	public int lastMotionIndex;
	
	public double avgKineticEnergy;
	
	public boolean isActive;
	public String beforeState;
	public String afterState;
	
	public MotionAnnotation annotation;
	
	
	public Matrix4d transform;
	
	public Matrix4d transformInverse;
	public Matrix4d startOpposite;
	public Matrix4d lastOpposite;
	public MotionClip opposite;
	
	private MotionClip(){
	}
	
	public int interactionMotionIndex(){
		return startMotionIndex + (interactionFrame - startFrame);
	}
	
	public int length(){
		return endFrame - startFrame;
	}
	
	public MotionData getMotionData(Motion[] totalMotions){
		return getMotionData(totalMotions, 0);
	}
	public MotionData getMotionData(Motion[] totalMotions, int margin){
		ArrayList<Motion> list = new ArrayList<Motion>();
		for (int i = startMotionIndex - margin; i <= lastMotionIndex + margin; i++) {
			list.add(totalMotions[i]);
		}
		return new MotionData(list);
	}
	
	public static boolean isMatching(MotionClip label1, MotionClip label2){
		return label1.oppositeFile != null && label1.oppositeFile.equals(label2.file) 
				&& label1.interactionFrame == label2.interactionFrame;
	}
	
	public String toString(){
		return Utils.toString(annotation.type, file, startFrame, endFrame);
	}
	
	private static boolean check(MGraph graph, MotionClip clip){
		if (clip == null) return true;
		
		MDatabase database = graph.getDatabase();
		Motion m1 = database.findMotion(clip.file + ".bvh", clip.startFrame);
		Motion m2 = database.findMotion(clip.file + ".bvh", clip.endFrame);
		MGraphNode start = graph.getNodeByMotion(m1);
		MGraphNode end = graph.getNodeByMotion(m2);
		if (start == null || end == null) return false;
		
		clip.startMotionIndex = m1.motionIndex;
		clip.lastMotionIndex = m2.motionIndex;
		
		if (end.index - start.index != m2.motionIndex - m1.motionIndex){
			return false;
		}
		
		MGraphEdge[] adjacentEdges = graph.getAdjacentEdgeList();
		double kineticEnergy = 0;
		for (int i = start.index; i < end.index-1; i++) {
			kineticEnergy += adjacentEdges[i].kineticEnergy;
		}
		clip.avgKineticEnergy = kineticEnergy / clip.length();
		
		clip.transform = calcClipTransform(clip, graph);
		clip.transformInverse = MathUtil.invert(clip.transform);
		return true;
	}
	
	private static Matrix4d calcClipTransform(MotionClip clip, MGraph graph){
		Matrix4d matrix = new Matrix4d();
		matrix.setIdentity();
		MGraphEdge[] adjEdges = graph.getAdjacentEdgeList();
		for (int i = clip.startMotionIndex; i < clip.lastMotionIndex; i++) {
			MGraphEdge edge = adjEdges[graph.getNodeByMotion(i).index];
			matrix.mul(edge.transform);
		}
		return matrix;
	}
	
	
	private static void setState(MotionClip clip, MotionAnnotation ann, boolean isActive){
		clip.isActive = isActive;
		if (isActive){
			clip.beforeState = ann.beforeActiveState;
			clip.afterState = ann.afterActiveState;
		} else {
			clip.beforeState = ann.beforePassiveState;
			clip.afterState = ann.afterPassiveState;
		}
	}
	
	public static ArrayList<MotionClip> clipList = new ArrayList<MotionClip>();
	public static HashMap<MotionClip, Integer> clipIndexMap = new HashMap<MotionClip, Integer>();
	public static HashMap<MotionAnnotation, MotionClip[]> getClipMapping(MGraph graph, ArrayList<MotionAnnotation> annList){
		if (annList == null) return null;
		Motion[] totalMotions = graph.getDatabase().getMotionList();
		ArrayList<MotionAnnotation> remainList = new ArrayList<MotionAnnotation>();
		HashMap<MotionAnnotation, MotionClip[]> clipMapping = new HashMap<MotionAnnotation, MotionClip[]>();
		for (MotionAnnotation ann : annList){
			MotionClip clip = new MotionClip();
			int oppositePerson = ann.oppositePerson;
			if (oppositePerson == 0){
				oppositePerson = (ann.person % 2) + 1; 
			}
			clip.file = ann.file.substring(0, ann.file.length() - ".bvh".length());
			clip.oppositeFile = clip.file.substring(0, clip.file.length() - 1) + oppositePerson;
			clip.startFrame = ann.startFrame;
			clip.endFrame = ann.endFrame;
			clip.interactionFrame = ann.interactionFrame;
			
			
			MotionClip oppositeClip = new MotionClip();
			oppositeClip.file = clip.oppositeFile;
			oppositeClip.oppositeFile = clip.file;
			oppositeClip.startFrame = clip.startFrame;
			oppositeClip.endFrame = clip.endFrame;
			oppositeClip.interactionFrame = ann.interactionFrame;
			
			clip.annotation = oppositeClip.annotation = ann; 
			setState(clip, ann, true);
			setState(oppositeClip, ann, false);
			
			if (ann.interactionFrame < 0){
				clip.interactionFrame = oppositeClip.interactionFrame = (ann.startFrame + ann.endFrame)/2;
			}
			if (ann.isAlone()){
				oppositeClip = null;
			}
			
			if (!check(graph, clip) || !check(graph, oppositeClip))  continue;
			
			remainList.add(ann);
			clipMapping.put(ann, new MotionClip[]{ clip, oppositeClip });
			if (ann.isAction()){
				clipIndexMap.put(clip, clipIndexMap.size());
				clipList.add(clip);
				if (oppositeClip != null){
					clipIndexMap.put(oppositeClip, clipIndexMap.size());
					clipList.add(oppositeClip);
					
					Motion s1 = totalMotions[clip.startMotionIndex];
					Motion s2 = totalMotions[oppositeClip.startMotionIndex];
					clip.startOpposite = getTransform(s1, s2);
					oppositeClip.startOpposite = getTransform(s2, s1);
					
					Motion l1 = totalMotions[clip.lastMotionIndex];
					Motion l2 = totalMotions[oppositeClip.lastMotionIndex];
					clip.lastOpposite = getTransform(l1, l2);
					oppositeClip.lastOpposite = getTransform(l2, l1);
					
					clip.opposite = oppositeClip;
					oppositeClip.opposite = clip;
				}
			}
		}
		
		annList.clear();
		annList.addAll(remainList);
		
		return clipMapping;
	}
	
	private static Matrix4d getTransform(Motion source, Motion target){
		Matrix4d t1 = MotionTransform.getPlaneTransform(source.root());
		Matrix4d t2 = MotionTransform.getPlaneTransform(target.root());
		t1.invert();
		t2.mul(t1, t2);
		return t2;
	}
}
