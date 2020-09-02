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
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.MainViewerModule.MainViewer;
import mrl.motion.viewer.module.Pose2dEditModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;

public class WalkTestModule extends Module{

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
		MotionDataConverter.setAllJoints();
//		OgreJNI.open(new double[]{ 0 });
//		MotionDataConverter.setNoBall();
//		MotionDataConverter.setNoBallVelocity();
		
		MotionDataConverter.setUseOrientation();
		String folder = "drb_e128k_tf";
//		String folder = "drb_e2k";
//		String folder = "drb_origin";
//		String folder = "dribble_no_ball";
//		String folder = "drb_graph_edit";
//		
//		String folder = "walk_gru";
		python = new RNNPython(folder, false);
		normal = new Normalizer(folder);
		
//		ArrayList<Double> distList = new ArrayList<Double>();
//		for (double[] x : normal.xList){
//			x = normal.deNormalizeX(x);
//			Vector2d v = new Vector2d(x[0], x[1]);
//			distList.add(v.length());
//		}
//		System.out.println(Arrays.toString(MathUtil.getStatistics(Utils.toDoubleArray(distList))));
//		System.exit(0);
		
		g = new RuntimeMotionGenerator();
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("Origin", Pose2d.BASE, new ItemDescription(new Vector3d(1, 0, 0)));
//		getModule(Pose2dEditModule.class);
		
		addMenu("&Menu", "Test &Target\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				start();
			}
		});
		addMenu("&Menu", "Test &Stop\tCtrl+S", SWT.MOD1 + 'S', new Runnable() {
			@Override
			public void run() {
				isStop = true;
				System.out.println("stop--------------");
				if (totalMotion2 != null && totalMotion2.size() > 0){
					getModule(ItemListModule.class).addSingleItem("TotalMotion2", new MotionData(totalMotion2));
				} else {
					getModule(ItemListModule.class).addSingleItem("TotalMotion", totalMotion);
				}
				getModule(ItemListModule.class).addSingleItem("TotalBall", totalBall, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS));
			}
		});
	}
	
	boolean isStop = false;
	Point3d ballPos = null;
	Point3d prevBallPos = null;
	
	
	public void start(){
		double[] initialY = normal.yList.get(3);
//		for (int i = 0; i < initialY.length; i++) {
//			initialY[i] *= 100;
//		}
		initialY = new double[initialY.length];
		python.model.setStartMotion(initialY);
		
		MainViewerModule mainViewer = getModule(MainViewerModule.class);
		long startTime = System.currentTimeMillis();
		
		MotionTransform t = new MotionTransform();
		PointIKSolver solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
		
		dummyParent().getDisplay().timerExec(1, new Runnable() {
			int frame = 0;
			boolean isFirst = false;
			@Override
			public void run() {
				if (dummyParent().isDisposed()) return;
				if (isStop) return;
				
				MotionData mData = null;
				while (true){
					int dt = (int)(System.currentTimeMillis() - startTime);
					int tIndex = dt/33;
					if (frame > tIndex) break;
					frame++;
					
					
					Point3d target3d;
					if (OgreJNI.isOpened()){
						OgreStatus status = OgreJNI.getStatus();
						target3d = status.mouse;
					} else {
						target3d = mainViewer.getPickPoint();
						if (target3d == null) break;
					}
					Point2d target = Pose2d.to2d(target3d);
//					System.out.println("pp : " + target3d + " : " + target);
					target = g.pose.globalToLocal(target);
					Vector2d v = new Vector2d(target);
					
					double lenLimit = maxLen/2;
					Vector2d n = new Vector2d(v);
					n.normalize();
					lenLimit += lenLimit + maxLen/2*Math.max(n.x, 0);
					
					if (v.length() > maxLen){
						v.scale(maxLen/v.length());
					}
//					v.scale(20);
//					v.scale(1d/20);
					
//					int time = getModule(Pose2dEditModule.class).getTime();
//					double movement = (time-100)/70d;
//					double mv = normal.xMeanAndStd[0][2] + movement*normal.xMeanAndStd[1][2];
					
					double[] x = new double[]{ v.x, v.y };
					x = normal.normalizeX(x);
					double[] output = python.model.predict(x);
					output = normal.deNormalizeY(output);
					
					
					PositionResultMotion motion = g.update(output);
//					mainViewer.addCameraTracking(g.pose.position3d());
					totalMotion.addAll(motion);
//					getModule(ItemListModule.class).addSingleItem("Motion", motion);
					getModule(ItemListModule.class).addSingleItem("Target", target3d, new ItemDescription(new Vector3d(0, 1, 0)));
					
					
					HashMap<String, Point3d> map = MotionDataConverter.dataToPointMap(output);
//					Motion mm = solver.solve(map, g.pose);
//					mm.ballContact = motion.get(0).ballContact;
//					mm.isLeftFootContact = motion.get(0).footContact.left;
//					mm.isRightFootContact = motion.get(0).footContact.right;
					Motion mm = Utils.last(g.motionList);
					mData = new MotionData(Utils.singleList(mm));
					totalMotion2.add(mm);
					getModule(ItemListModule.class).addSingleItem("Motion2", mData);
					
					if (MotionDataConverter.includeBall){
						double[] b = new double[3];
						System.arraycopy(output, 0, b, 0, 3);
						Point3d p = new Point3d(b[0], 0, b[2]);
						p = Pose2d.to3d(g.pose.localToGlobal(Pose2d.to2d(p)));
						p.y = b[1];
						getModule(ItemListModule.class).addSingleItem("ball", p, BallTrajectoryGenerator.ballDescription());
	//					System.out.println("b contact :: " + String.format("%.4f, %.4f", output[6], output[7]) + " :: " + p);
					
						if (prevBallPos != null && ballPos != null){
							double v0 = ballPos.y - prevBallPos.y;
							double v1 = p.y - ballPos.y;
							if (v0 < 0 && v1 > 0){
								System.out.println("bounce :: " + frame + " : " + Utils.toString(String.format("%.4f, %.4f", output[6], output[7]), prevBallPos, ballPos, p));
							}  else {
								System.out.println("  non :: " + frame + " : " + Utils.toString(String.format("%.4f, %.4f", output[6], output[7]), prevBallPos, ballPos, p));
							}
						}
						prevBallPos = ballPos;
						ballPos = new Point3d(p);
						totalBall.add(ballPos);
					}
				}
				if (mData != null){
					OgreJNI.setMotion(new MotionData[] { mData });
				}
				if (ballPos != null) OgreJNI.setBall(Utils.singleList(ballPos));
				dummyParent().getDisplay().timerExec(1, this);
			}
		});
	}
	
	

	public static void main(String[] args) {
		MainApplication.run(new WalkTestModule());
		OgreJNI.close();
	}
}
