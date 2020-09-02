package mrl.motion.neural.run;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.neural.tennis.TennisDataGenerator;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.MainViewerModule.MainViewer;
import mrl.motion.viewer.module.Pose2dEditModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;

public class NoInputTestModule extends Module{

	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	private Normalizer jointNormal;
	
	private PositionResultMotion totalMotion = new PositionResultMotion();
	private ArrayList<Motion> totalMotion2 = new ArrayList<Motion>();
	private TimeBasedList<Point3d> totalBall = new TimeBasedList<Point3d>();
	
//	double maxLen = 180;
	double maxLen = 250;
	
	@Override
	protected void initializeImpl() {
//		OgreJNI.open(new double[]{ 0 });
		
//		MotionDataConverter.setAllJoints();
//		String folder = "bs_d20d";
		
		MotionDataConverter.setNoBall();
//		TennisDataGenerator.setTennisJointSet();
		String folder = "tnr_d01";
//		String folder = "tn_d04";
//		
		python = new RNNPython(folder, false);
		normal = new Normalizer(folder);
		
		g = new RuntimeMotionGenerator();
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("Origin", Pose2d.BASE, new ItemDescription(new Vector3d(1, 0, 0)));
		
		generate(1000);
	}
	
	boolean isStop = false;
	Point3d ballPos = null;
	Point3d prevBallPos = null;
	
	
	public void generate(int size){
		double[] initialY = normal.yList.get(3);
		initialY = new double[initialY.length];
		python.model.setStartMotion(initialY);
		double[] prevY = MathUtil.copy(initialY);
		
		
		MotionTransform t = new MotionTransform();
		PointIKSolver solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
		
		
		for (int i = 0; i < size; i++) {
			double[] x = normal.xList.get(0);
			double[] output = python.model.predict(x);
			output = normal.deNormalizeY(output);
			
			if (RNNDataGenerator.USE_RESIDUAL){
				for (int j = MotionDataConverter.ROOT_OFFSET; j < output.length; j++) {
					output[j] += prevY[j];
				}
				prevY = output;
				python.model.setStartMotion(prevY);
			}
			
			PositionResultMotion motion = g.update(output);
//			mainViewer.addCameraTracking(g.pose.position3d());
			totalMotion.addAll(motion);
//			getModule(ItemListModule.class).addSingleItem("Motion", motion);
			
			if (MotionDataConverter.KeyJointList.length > 13){
				HashMap<String, Point3d> map = MotionDataConverter.dataToPointMap(output);
				Motion mm = solver.solve(map, g.pose);
				mm.ballContact = motion.get(0).ballContact;
				mm.isLeftFootContact = motion.get(0).footContact.left;
				mm.isRightFootContact = motion.get(0).footContact.right;
				totalMotion2.add(mm);
			}
			
			if (MotionDataConverter.includeBall){
				double[] b = new double[3];
				System.arraycopy(output, 0, b, 0, 3);
				Point3d p = new Point3d(b[0], 0, b[2]);
				p = Pose2d.to3d(g.pose.localToGlobal(Pose2d.to2d(p)));
				p.y = b[1];
				prevBallPos = ballPos;
				ballPos = new Point3d(p);
				totalBall.add(ballPos);
			}
		}
		
		if (totalMotion2 != null && totalMotion2.size() > 0){
			getModule(ItemListModule.class).addSingleItem("TotalMotion2", new MotionData(totalMotion2));
		} else {
			getModule(ItemListModule.class).addSingleItem("TotalMotion", totalMotion);
		}
		getModule(ItemListModule.class).addSingleItem("TotalBall", totalBall, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS));
	}
	
	

	public static void main(String[] args) {
		MainApplication.run(new NoInputTestModule());
		OgreJNI.close();
	}
}
