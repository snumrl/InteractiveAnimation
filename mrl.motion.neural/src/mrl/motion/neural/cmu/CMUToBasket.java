package mrl.motion.neural.cmu;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.eclipse.swt.events.DisposeEvent;
import org.eclipse.swt.events.DisposeListener;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.SkeletonData.Joint;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.position.PositionMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.MathUtil;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

public class CMUToBasket extends Module{
	
	static String[] JointMapping = new String[]{
		"Head_End",
		"LeftHand",
		"LeftFoot",
		"LeftToeBase",
		"RightHand",
		"RightFoot",
		"RightToeBase",
		
		"LeftArm",
		"RightArm",
		
		"LeftForeArm",
		"LeftLeg",
		"RightForeArm",
		"RightLeg",
		
		"Spine",
		"LeftHandIndex1",
		"RightHandIndex1",
		"Spine1",
		"LeftUpLeg",
		"RightUpLeg",
	};
	

	@Override
	protected void initializeImpl() {
		
//		MotionData cmu = new BVHParser().parse(new File("walkData\\motion\\16_08_1.bvh"));
		MotionData cmu = new BVHParser().parse(new File("walkMotion\\69_04_1.bvh"));
//		MotionData cmu = new BVHParser().parse(new File("walkMotion\\69_03_1.bvh"));
		scale(cmu, 0.88);
//		cmu.skeletonData.root
		Vector3d t = MathUtil.getTranslation(cmu.motionList.get(12).root());
		TimeBasedList<ArrayList<Point3d>> ppList = new TimeBasedList<ArrayList<Point3d>>();
		ArrayList<PositionMotion> pmList = new ArrayList<PositionMotion>();
		for (Motion motion : cmu.motionList){
			ArrayList<Point3d> pList = new ArrayList<>();
			pmList.add(new PositionMotion(Motion.mirroredMotion(motion)));
//			pmList.add(new PositionMotion(motion, false));
			pList.addAll(Motion.getPointData(cmu.skeletonData, motion).values());
			ppList.add(pList);
		}
		
		MotionTransform mt = new MotionTransform();
		PointIKSolver s = new PointIKSolver(mt.skeletonData, mt.sampleMotion);
		ArrayList<Motion> solved = new ArrayList<Motion>();
		
		for (PositionMotion pm : pmList){
			HashMap<String, Point3d> mapped = new HashMap<String, Point3d>();
			for (int i = 0; i < JointMapping.length; i++) {
				mapped.put(MotionDataConverter.KeyJointList[i], pm.pointData.get(JointMapping[i]));
			}
			mapped.put("Spine", pm.pointData.get("Hips"));
			Point3d hip = new Point3d();
			hip.add(pm.pointData.get("LeftUpLeg"), pm.pointData.get("RightUpLeg"));
			hip.scale(0.5);
			mapped.put("Hips", hip);
			solved.add(s.solve(mapped, pm.pose));
		}
		
//		MotionData basket = new BVHParser().parse(new File("basketData\\motion\\s_003_1_1.bvh"));
//		
//		for (Motion motion : basket.motionList){
//			motion.root().m03 = t.x;  
//			motion.root().m23 = t.z;  
//		}
		MotionData basket = new MotionData(mt.skeletonData, solved);
		FootContactDetection.checkFootContact(basket);
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("basket", basket);
		getModule(ItemListModule.class).addSingleItem("cmu", ppList);
		
//		OgreJNI.open(new double[]{ 0 });
//		OgreJNI.setMotion(new MotionData[]{ basket });
	}
	
	public static ArrayList<Motion> convert(String cmuFile, boolean mirror){
		MotionData cmu = new BVHParser().parse(new File(cmuFile));
		scale(cmu, 0.88);
		ArrayList<PositionMotion> pmList = new ArrayList<PositionMotion>();
		for (Motion motion : cmu.motionList){
			if (mirror){
				pmList.add(new PositionMotion(Motion.mirroredMotion(motion)));
			} else {
				pmList.add(new PositionMotion(motion));
			}
		}
		
		MotionTransform mt = new MotionTransform();
		PointIKSolver s = new PointIKSolver(mt.skeletonData, mt.sampleMotion);
		ArrayList<Motion> solved = new ArrayList<Motion>();
		
		for (PositionMotion pm : pmList){
			HashMap<String, Point3d> mapped = new HashMap<String, Point3d>();
			for (int i = 0; i < JointMapping.length; i++) {
				mapped.put(MotionDataConverter.KeyJointList[i], pm.pointData.get(JointMapping[i]));
			}
			mapped.put("Spine", pm.pointData.get("Hips"));
			Point3d hip = new Point3d();
			hip.add(pm.pointData.get("LeftUpLeg"), pm.pointData.get("RightUpLeg"));
			hip.scale(0.5);
			mapped.put("Hips", hip);
			solved.add(s.solve(mapped, pm.pose));
		}
		
		for (int i = 0; i < solved.size() - 1;i++) {
			Motion m1 = solved.get(i);
			Motion m2 = solved.get(i+1);
			m1.frameIndex = i;
			m2.frameIndex = i+1;
			m1.next = m2;
			m2.prev = m1;
		}
		FootContactDetection.CONTACT_MARGIN = 2;
		FootContactDetection.checkFootContact(new MotionData(mt.skeletonData, solved));
		return solved;
		
	}
	
	
	private static void scale(MotionData mData, double scale){
		
		SkeletonData skeleton = mData.skeletonData;
		skeleton.get("LeftUpLeg").transition.x = 10;
		skeleton.get("RightUpLeg").transition.x = -10;
		
//		skeleton.get("LeftUpLeg").transition.z = 0;
//		skeleton.get("RightUpLeg").transition.z = 0;
//		skeleton.get("LeftLeg").transition.x = 15.25;
//		skeleton.get("LeftLeg").transition.y = -41.9;
		
		skeleton.get("LeftArm").transition.x = 24;
		skeleton.get("LeftArm").transition.y = 8.2;
		skeleton.get("RightArm").transition.x = -24;
		for (Joint j : skeleton.values()){
			scale(j, scale);
		}
		for (Motion motion : mData.motionList){
			Vector3d t = MathUtil.getTranslation(motion.root());
			t.scale(scale);
			motion.root().setTranslation(t);
		}
		
		double spine = 1.88;
		scale(skeleton.get("Spine"), spine);
		scale(skeleton.get("Spine1"), spine);
		
		double arm1 = 1.02;
		scale(skeleton.get("LeftForeArm"), arm1);
		scale(skeleton.get("RightForeArm"), arm1);
		double arm2 = 1.52;
		scale(skeleton.get("LeftHand"), arm2);
		scale(skeleton.get("RightHand"), arm2);
		
		scale(skeleton.get("Head"), 4);
		
		
	}
	
	private static void scale(Joint j, double scale){
		j.transition.scale(scale);
		j.length = j.transition.length();
	}
	
	

	
	public static void main(String[] args) {
		MainApplication.run(new CMUToBasket());
		OgreJNI.close();
	}
}
