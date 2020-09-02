package mrl.motion.neural.figure;

import java.util.ArrayList;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.position.PositionMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.MainViewerModule.MainViewer;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

public class MotionAugmentationSample extends Module {

	public ArrayList<MotionSegment> originList;
	public ArrayList<MotionSegment> editedList;
	
	private ArrayList<ArrayList<MotionData>> subList = new ArrayList<ArrayList<MotionData>>();
	
	public MotionAugmentationSample(TrajectoryEditParam edit){
		originList = edit.originList;
		editedList = edit.editedList;
	}
	
	private MotionSegment merged(ArrayList<MotionSegment> list, int start){
		MotionSegment merged = null;
		for (int i = start; i < list.size()-1; i++) {
			merged = MotionSegment.stitch(merged, new MotionSegment(list.get(i)), true);
		}
		return merged;
	}
	
	@Override
	protected void initializeImpl() {
		MainViewer viewer = getModule(MainViewerModule.class).getMainViewer();
		viewer.eye = new Vector3d(425.01548341376144, 462.05914752442936, 211.23141449699625);
		viewer.center = new Vector3d(-160.44265801422856, -9.582490170457202, 164.88540260744972);
		viewer.upVector = new Vector3d(-0.6017782703613126, 0.7581213862976698, -0.2512267441158342);
//		viewer.drawPlane = false;
//		viewer.backgroundColor = 1;
		
		MotionSegment current = new MotionSegment(originList.get(0), 0, 0);
		Matrix4d t = new Matrix4d();
		t.rotY(-Math.PI/2);
		for (Motion m : current.getEntireMotion()){
			m.root().mul(t, m.root());
		}
		current.updateNotBlendedAsCurrent();
		
		
		ArrayList<MotionSegment> oList = new ArrayList<MotionSegment>();
		ArrayList<MotionSegment> eList = new ArrayList<MotionSegment>();
		ArrayList<Integer> lenList = new ArrayList<Integer>();
		for (int i = 0; i < originList.size()-1; i++) {
//			MotionSegment origin = new MotionSegment(originList.get(i));
//			MotionSegment edited = new MotionSegment(editedList.get(i));
			MotionSegment origin = merged(originList, i);
			MotionSegment edited = new MotionSegment(editedList.get(i));
			lenList.add(edited.length());
			MotionSegment.align(current, origin);
			MotionSegment.align(current, edited);
			oList.add(new MotionSegment(origin));
			eList.add(new MotionSegment(edited));
			current = MotionSegment.stitch(current, new MotionSegment(editedList.get(i)), true);
		}
		
		int interval = 15;
		for (int i = 0; i < oList.size(); i++) {
			addMotion(oList.get(i), new Vector3d(0.8, 0.8, 1), interval);
			addMotion(eList.get(i), new Vector3d(1, 0.8, 0.8), interval);
		}
		
		double sc = 0.2;
		ArrayList<Pose2d> poseList1 = new ArrayList<Pose2d>();
		Vector3d oColor = new Vector3d(sc, sc, 1);
		for (int i = 0; i < oList.size(); i++) {
			Motion motion = oList.get(i).getMotionList().get(lenList.get(i) - 1);
			poseList1.add(PositionMotion.getPose(motion));
			addMotion(motion, oColor);
		}
		getModule(ItemListModule.class).addSingleItem("poseList1", poseList1, new ItemDescription(oColor));
		
		ArrayList<Pose2d> poseList2 = new ArrayList<Pose2d>();
		Vector3d eColor = new Vector3d(1, sc, sc);
		for (int i = 0; i < eList.size(); i++) {
			Motion motion = eList.get(i).firstMotion();
			poseList2.add(PositionMotion.getPose(motion));
			addMotion(motion, eColor);
		}
		{
			Motion motion = Utils.last(eList).lastMotion();
			poseList2.add(PositionMotion.getPose(motion));
			addMotion(motion, eColor);
		}
		getModule(ItemListModule.class).addSingleItem("poseList", poseList2, new ItemDescription(eColor));
	}
	
	int mIndex = 0;
	private void addMotion(MotionSegment s, Vector3d color, int interval){
		ArrayList<MotionData> motionList = new ArrayList<MotionData>();
		ArrayList<Motion> sMotion = s.getMotionList();
		
		for (int i = 0; i < sMotion.size(); i+=interval) {
			if (i == 0) continue;
			motionList.add(new MotionData(Utils.singleList(sMotion.get(i))));
		}
		
		ArrayList<Point3d> points = new ArrayList<Point3d>();
		for (int i = 0; i < sMotion.size(); i+=5) {
			if (i == 0) continue;
			Vector3d t = MathUtil.getTranslation(sMotion.get(i).root());
			t.y = 0;
			points.add(new Point3d(t));
		}
		
		mIndex++;
		getModule(ItemListModule.class).addSingleItem("Motion" + mIndex, motionList, new ItemDescription(color));
		getModule(ItemListModule.class).addSingleItem("Root" + mIndex, points, new ItemDescription(color));
		subList.add(motionList);
	}
	
	private void addMotion(Motion motion, Vector3d color){
		for (ArrayList<MotionData> sList : subList){
			for (int i = 0; i < sList.size(); i++) {
				if (sList.get(i).motionList.get(0) == motion){
					sList.remove(i);
					break;
				}
			}
		}
		
		MotionData mData = new MotionData(Utils.singleList(motion));
		mIndex++;
		getModule(ItemListModule.class).addSingleItem("Motion" + mIndex, mData, new ItemDescription(color));
	}
	
	public static void main(String[] args) {
		MathUtil.random.setSeed(38);
		TrajectoryEditParam edit = new BasketDataGenerator().generateDribble();
		MainApplication.run(new MotionAugmentationSample(edit));
	}
}
