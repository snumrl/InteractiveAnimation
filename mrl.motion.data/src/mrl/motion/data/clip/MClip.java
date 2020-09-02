package mrl.motion.data.clip;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionAnnotation;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.position.PositionMotion;
import mrl.util.MathUtil;

public class MClip {
	
	public static final int MARGIN = 10;
	
	public int index;

	public String file;
	public int startFrame;
	public int endFrame;
	public int length;
	public int interactionOffset;
	
	public int startMotionIndex;
	public int lastMotionIndex;
	
	public String type;
	
	private int oppositePerson;
	public MClip opposite;
	public double angleOffset = Double.NaN;
	
	private Matrix4d[][] transformMap;
	private Matrix4d[] startToInteraction;
	private Matrix4d[] interactionToEnd;
	private MDatabase database;
	
	public boolean isActionClip;
	
	public Pose2d transform;
	
	public int interactionMotionIndex(){
		return startMotionIndex + interactionOffset;
	}
	
	public static ArrayList<MClip> getClipList(MDatabase database, ArrayList<MotionAnnotation> annList){
		Motion[] mList = database.getMotionList();
		ArrayList<MClip> list = new ArrayList<MClip>();
		HashMap<String, ArrayList<MClip>> clipMap = new HashMap<String, ArrayList<MClip>>();
		HashMap<String, Integer> types = new HashMap<String, Integer>();
		int sum = 0;
		HashSet<MClip> toRemove = new HashSet<MClip>();
		for (MotionAnnotation ann : annList){
			MClip clip = new MClip();
			clip.oppositePerson = ann.oppositePerson;
			
			clip.database = database;
			clip.file = ann.file.substring(0, ann.file.length() - ".bvh".length());
			clip.startFrame = ann.startFrame;
			clip.endFrame = ann.endFrame;
			clip.length = clip.endFrame - clip.startFrame;
			sum+=clip.length+1;
			
			clip.type = ann.type;
			if(types.containsKey(clip.type)) {
				types.put(clip.type, types.get(clip.type) + 1);
			}
			else {
				types.put(clip.type, 1);
			}
			
			if (ann.interactionFrame >= 0){
				clip.interactionOffset = ann.interactionFrame - ann.startFrame;
				clip.isActionClip = true; 
			} else {
				clip.interactionOffset = (ann.endFrame - ann.startFrame)/2;
				clip.isActionClip = false; 
			}
			
			Motion m1 = database.findMotion(clip.file + ".bvh", clip.startFrame);
			Motion m2 = database.findMotion(clip.file + ".bvh", clip.endFrame);
			clip.startMotionIndex = m1.motionIndex;
			clip.lastMotionIndex = m2.motionIndex;
			
			if (ann.isInclude()){
//				clip.index = list.size();
				list.add(clip);
			} else {
				clip.index = -1;
				list.add(clip);
				toRemove.add(clip);
			}
			
			int n = MARGIN * 2 + 1;
			clip.transformMap = new Matrix4d[n][n];
			clip.startToInteraction = new Matrix4d[n];
			clip.interactionToEnd = new Matrix4d[n];
			for (int i = 0; i < n; i++) {
				int startOffset = i - MARGIN;
				for (int j = 0; j < n; j++) {
					int endOffset = j - MARGIN;
					clip.transformMap[i][j] = motionTransform(
							mList[clip.startMotionIndex + startOffset], 
							mList[clip.lastMotionIndex + endOffset]);
				}
				clip.startToInteraction[i] = motionTransform(
						mList[clip.startMotionIndex + startOffset], 
						mList[clip.startMotionIndex + clip.interactionOffset]);
				clip.interactionToEnd[i] = motionTransform(
						mList[clip.startMotionIndex + clip.interactionOffset], 
						mList[clip.lastMotionIndex + startOffset]);
			}
			
			
			ArrayList<MClip> cList = clipMap.get(clip.file);
			if (cList == null){
				cList = new ArrayList<MClip>();
				clipMap.put(clip.file, cList);
			}
			cList.add(clip);
			
			Pose2d p1 = PositionMotion.getPose(mList[clip.startMotionIndex]);
			Pose2d p2 = PositionMotion.getPose(mList[clip.lastMotionIndex+1]);
			clip.transform = Pose2d.relativePose(p1, p2);
//			double move = MathUtil.length(clip.transform.position)/clip.length();
//			System.out.println(clip + "\t\t" + move);
			
//			System.out.println(clip + " : " + clip.length());
		}
		System.out.println("total frame : "  + (annList.size()) + ", "+ sum + " :: types=" + types.size());
		for(String key : types.keySet()) {
			System.out.println(key + " : " + types.get(key));
		}
		
		System.out.println("--- check pass angle ---");
		for (MClip clip : list){
			if (clip.type.startsWith("shoot")){
				Matrix4d t1 = mList[clip.startMotionIndex + clip.interactionOffset].root();
				Vector3d p1 = MathUtil.getTranslation(t1);
				// basket position( when motion captured )
				Vector3d p2 = new Vector3d();
				
				Vector2d v = new Vector2d(p2.x - p1.x, p2.z - p1.z);
				clip.angleOffset = Math.atan2(-v.y, v.x);
				continue;
			}
			
			
			if (clip.oppositePerson == 0) continue;
			MClip opposite = findOpposite(clip, clipMap);
			if (clip.type.equals("pass")){
				if (!opposite.type.equals("pass'")){
					throw new RuntimeException(clip + " : " + opposite);
				}
			} else if (clip.type.equals("pass'")){
				if (!opposite.type.equals("pass")){
					throw new RuntimeException(clip + " : " + opposite);
				}
			} else if (clip.type.startsWith("screen")){
				if (!opposite.type.equals("screen'")){
					throw new RuntimeException(clip + " : " + opposite);
				}
			} else if (clip.type.startsWith("screen'")){
				if (!opposite.type.equals("screen")){
					throw new RuntimeException(clip + " : " + opposite);
				}
			} else if (clip.type.equals("pass_long")){
				if (!opposite.type.equals("pass'")){
					throw new RuntimeException(clip + " : " + opposite);
				}
			} else {
				throw new RuntimeException(clip + " : " + opposite);
			}
			clip.opposite = opposite;
			opposite.opposite = clip;
			
			Matrix4d t1 = mList[clip.startMotionIndex + clip.interactionOffset].root();
			Matrix4d t2 = mList[opposite.startMotionIndex + opposite.interactionOffset].root();
			
			Vector3d p1 = MathUtil.getTranslation(t1);
			Vector3d p2 = MathUtil.getTranslation(t2);
			
			Vector2d v = new Vector2d(p2.x - p1.x, p2.z - p1.z);
			clip.angleOffset = Math.atan2(-v.y, v.x);
			v.scale(-1);
			opposite.angleOffset = Math.atan2(-v.y, v.x);
			if (Double.isNaN(clip.angleOffset) || Double.isNaN(opposite.angleOffset)){
				System.out.println(clip + " , " + opposite);
				throw new RuntimeException();
			}
		}
		
		ArrayList<MClip> remained = new ArrayList<MClip>();
		for (MClip clip : list){
			if (toRemove.contains(clip)) continue;
			remained.add(clip);
		}
		list = remained;
		
		for (int i = 0; i < list.size(); i++) {
			list.get(i).index = i;
		}
		return list;
	}
	
	public static MClip findOpposite(MClip clip, HashMap<String, ArrayList<MClip>> clipMap){
		if (clip.oppositePerson == 0) throw new RuntimeException();
		String file = clip.file.substring(0, clip.file.length()-1) + clip.oppositePerson;
		ArrayList<MClip> list = clipMap.get(file);
		for (MClip c : list){
			if (!c.type.startsWith("pass") && !c.type.startsWith("screen")) continue;
			if (c.startFrame >= clip.startFrame && c.startFrame <= clip.endFrame) return c;
			if (c.endFrame >= clip.startFrame && c.endFrame <= clip.endFrame) return c;
			
			if (clip.startFrame >= c.startFrame && clip.startFrame <= c.endFrame) return c;
			if (clip.endFrame >= c.startFrame && clip.endFrame <= c.endFrame) return c;
			
//			if (Math.abs(c.startFrame - c.startFrame) <= 2) return c;
//			if (Math.abs(c.endFrame - clip.endFrame) <= 2) return c;
		}
		throw new RuntimeException(clip.toString());
	}
	
	public Matrix4d getTransformBetween(int from, int to) {
		if (to >= length + MARGIN) to = length + MARGIN;
		
		if (from <= MARGIN && to >= length - MARGIN) {
			return new Matrix4d(transformMap[from + MARGIN][to - length + MARGIN]);
		}
		Motion[] mList = database.getMotionList();
		return motionTransform(mList[startMotionIndex + from], mList[startMotionIndex + to]);
	}
	
	public Matrix4d getTransform(int startOffset, int endOffset){
		return transformMap[startOffset + MARGIN][endOffset + MARGIN];
	}
	
	public Matrix4d getStartToInteraction(int startOffset){
		return startToInteraction[startOffset + MARGIN];
	}
	
	public Matrix4d getInteractionToEnd(int endOffset){
		return interactionToEnd[endOffset + MARGIN];
	}
	
	public int length(){
		return endFrame - startFrame + 1;
	}
	
	public String toString(){
		return type + "(" + file + ":" + startFrame + ":" + index + ")"; 
	}
	
	public static Matrix4d motionTransform(Motion source, Motion target){
		Matrix4d t1 = MotionTransform.getPlaneTransform(source.root());
		Matrix4d t2 = MotionTransform.getPlaneTransform(target.root());
		t1.invert();
//		t2.mul(t2, t1);
		t2.mul(t1, t2);
		return t2;
	}
}
