package mrl.motion.neural.loco;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point3d;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.position.PositionMotion;
import mrl.motion.viewer.module.MotionListModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.util.Configuration;
import mrl.widget.app.Item;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;

public class LocomotionMotionViewer {

	public static void main(String[] args) {
		MainApplication app = new MainApplication();
		MotionListModule listModule = app.getModule(MotionListModule.class);
		MotionDataConverter.setCMUAllJoints();
		FootContactDetection.heightLimit = new double[]{ 20, 13, 11 };
		FootContactDetection.velocityLimit = new double[]{ 4, 4, 4 };
		
		Configuration.setDataFolder("locomotion\\gorilla");
		MDatabase database = MDatabase.load();
		database.addMirroredData();
		MotionData mData = database.getMotionDataList()[1];
		app.getModule(ItemListModule.class).addSingleItem("motion", mData);
		
		int dSize = database.getMotionDataList()[0].motionList.size();
		TimeBasedList<ArrayList<Point3d>> points = new TimeBasedList<ArrayList<Point3d>>();
		TimeBasedList<Pose2d> poseList = new TimeBasedList<Pose2d>();
		for (int i = 0; i < dSize; i++) {
			double d = database.getDist().getDistance(i, i+dSize);
			System.out.println("dist :: " + i + " : " + d);
			Motion motion = mData.motionList.get(i);
			HashMap<String, Point3d> pp = Motion.getPointData(SkeletonData.instance, motion);
			ArrayList<Point3d> pList = new ArrayList<Point3d>();
			pList.addAll(pp.values());
			points.add(pList);
			poseList.add(PositionMotion.getPose(motion));
		}
		app.getModule(ItemListModule.class).addSingleItem("points", points);
		app.getModule(ItemListModule.class).addSingleItem("poseList", poseList);
//		listModule.loadMotionFolder("locomotion\\indianDance\\motion");
//		listModule.loadMotionFolder("locomotion\\gorilla\\motion");
		
		Pose2d p = Pose2d.BASE;
		Item item = new Item(p);
		item.setLabel("Pose");
		app.getModule(ItemListModule.class).addItem(item);
		app.open();
	}
}
