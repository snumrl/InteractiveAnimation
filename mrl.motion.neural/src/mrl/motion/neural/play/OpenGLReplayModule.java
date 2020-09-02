package mrl.motion.neural.play;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.basket.BasketDataGenerator.ControlTarget;
import mrl.motion.neural.basket.BasketGraph;
import mrl.motion.neural.data.BasketTimeControl;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.loco.DirectionPredictModule.DPStatus;
import mrl.motion.position.PositionMotion;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.SWTViewableCanvas;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.util.FileUtil;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;
import mrl.widget.app.Item.ItemDescription;

import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.widgets.Display;

public class OpenGLReplayModule extends Module{

	private SWTViewableCanvas canvas;
	
	private int maxFrame = -1;
	
	@Override
	protected void initializeImpl() {
		canvas = getModule(MainViewerModule.class).getMainViewer();
		
//		MDatabase database = BasketDataGenerator.loadBasketData();
//		motionData = database.getMotionDataList()[0];
//		getModule(ItemListModule.class).addSingleItem("motion", motionData);
		
		presetLocomotion2("output\\loco_edin_dr_pre");
//		presetLocomotion("output\\loco_indian2");
//		presetRandomMotion(193, 465, true, true);
//		presetMotionCapture("s_009_1_%d.bvh", 849, 1323);
		
		addMenu("&Menu", "&Test\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				Replay replay = new Replay();
				replay.capture("D:\\data\\RNN\\output\\loco_edin_dr_pre\\capture");
			}
		});
		addMenu("&Menu", "&Test s\tCtrl+S", SWT.MOD1 + 'S', new Runnable() {
			@Override
			public void run() {
				Point target = new Point(1280, 720);
				while (true){
					Point cSize = canvas.getSize();
					Point sSize = canvas.getShell().getSize();
					int dx = target.x - cSize.x;
					int dy = target.y - cSize.y;
					if (dx == 0 && dy == 0){
						System.out.println("finished :: " + cSize + " : " + sSize);
						break;
					}
					canvas.getShell().setSize(sSize.x + dx, sSize.y + dy);
					canvas.getShell().layout();
					
					Display display = canvas.getDisplay();
					for (int i = 0; i < 30; i++) {
						display.readAndDispatch();
					}
				}
			}
		});
		
		canvas.getDisplay().timerExec(500, new Runnable() {
			@Override
			public void run() {
				canvas.getShell().setSize(1531, 852);
			}
		});
	}
	
	private void presetRandomMotion(int start, int end, boolean includeBall, boolean position){
		canvas.eye = new Vector3d(210.2968493374407, 195.16788978300897, -69.70737374437998);
		canvas.center = new Vector3d(-149.7387416125106, -9.738452452998443, -53.99307612964031);
		canvas.upVector = new Vector3d(-0.5022923284658074, 0.8646446295932869, -0.009595899117732763);
		
		MathUtil.random.setSeed(16254);
		BasketGraph.ALL_TIME = true;
		BasketDataGenerator g = new BasketDataGenerator();
		ArrayList<Motion> motion = g.generateMotion(100).getMotionList();
		
		BasketTimeControl control = new BasketTimeControl(g.database, g.targetList);
		control.includeRootMove = false;
		control.setData(motion, null);
		ArrayList<double[]> xList = new ArrayList<double[]>();
		for (int i = 0; i < end + 10; i++) {
			xList.add(control.getControl(i));
		}
		
		
		ArrayList<Point3d> ball = new BallTrajectoryGenerator().generate(motion);
		
		
		for (int i = 0; i < g.targetList.size()-1; i++) {
			ControlTarget t1 = g.targetList.get(i);
			if (t1.clip.type.startsWith("shoot") || t1.clip.type.equals("pass")){
				ControlTarget t2 = g.targetList.get(i+1);
//				System.out.println("tt : " + i + " : " + t1.clip.type + " : " + t1.mIndex + "-> " + t2.mIndex);
				for (int j = t1.mIndex+1; j < t2.mIndex; j++) {
					ball.set(j, null);
				}
			}
		}
		
		TimeBasedList<Point3d> bt = new TimeBasedList<Point3d>();
		bt.addAll(Utils.cut(ball, start, end));
		if (position){
			ArrayList<double[]> dataList = MotionDataConverter.motionToData(Utils.cut(motion, start, end), motion.get(start-1), false);
			Pose2d pose = PositionMotion.getPose(motion.get(start-1));
			Matrix4d mm = Pose2d.globalTransform(Pose2d.BASE, pose).to3d();
			PositionResultMotion pm = MotionDataConverter.dataToMotion(dataList, mm);
			getModule(ItemListModule.class).addSingleItem("Motion", pm);
			
			motion = Utils.cut(motion, start, end);
			xList = Utils.cut(xList, start, end);
			TimeBasedList<Object> targetList = new TimeBasedList<Object>();
			for (int dIdx = 0; dIdx < xList.size(); dIdx++) {
				double[] input = xList.get(dIdx);
				int i = input.length-6;
				System.out.println(dIdx + " : " + Arrays.toString(input));
				Pose2d p = new Pose2d(input[i], input[i+1], input[i+2], input[i+3]);
				p = PositionMotion.getPose(motion.get(dIdx)).localToGlobal(p);
				if (input[4] == 1){
					targetList.add(Pose2d.to3d(p.position));
				} else {
					targetList.add(p);
				}
			}
			ItemDescription des = new ItemDescription(new Vector3d(0.5, 1, 0.5));
			des.size = 10;
			getModule(ItemListModule.class).addSingleItem("Target", targetList, des);
		} else {
			motion = Utils.cut(motion, start, end);
			getModule(ItemListModule.class).addSingleItem("Motion", new MotionData(motion));
		}
		if (includeBall){
			getModule(ItemListModule.class).addSingleItem("Ball", bt, BallTrajectoryGenerator.ballDescription());
		}
		maxFrame = end - start + 1;
	}
	
	private void presetMotionCapture(String path, int startFrame, int endFrame){
		
		canvas.eye = new Vector3d(377.98403561258516, 239.929273150635, 69.75355259425797);
		canvas.center = new Vector3d(68.06177369785054, 15.871544909848982, 13.64084707505449);
		canvas.upVector = new Vector3d(-0.584008030446584, 0.8092135323237658, -0.06409430144710053);
		
		Vector3d[] colorList = new Vector3d[10];
		Vector3d basicColor = new Vector3d(0.9, 0.9, 0.3);
		for (int i = 0; i < colorList.length; i++) {
			colorList[i] = basicColor;
		}
		colorList[0] = new Vector3d(1, 0.8, 0.7);
		colorList[1] = new Vector3d(0.7, 1, 0.8);
		colorList[2] = new Vector3d(0.6, 0.6, 1);
		
		MDatabase database = BasketDataGenerator.loadBasketData();
		for (int i = 0; i < 10; i++) {
			String file = String.format(path, (i+1));
			int mIdx = database.findMotionDataIndex(file);
			if (mIdx < 0) break;
			MotionData mData = database.getMotionDataList()[mIdx];
			mData = new MotionData(Utils.cut(mData.motionList, startFrame, endFrame));
			ItemDescription des = new ItemDescription(colorList[i]);
			getModule(ItemListModule.class).addSingleItem(file, mData, des);
		}
		maxFrame = endFrame + startFrame + 1;
	}
	
	private void presetLocomotion(String folder){
		canvas.eye = new Vector3d(377.98403561258516, 239.929273150635, 69.75355259425797);
		canvas.center = new Vector3d(68.06177369785054, 15.871544909848982, 13.64084707505449);
		canvas.upVector = new Vector3d(-0.584008030446584, 0.8092135323237658, -0.06409430144710053);
		
		PositionResultMotion totalMotion = (PositionResultMotion)FileUtil.readObject(folder + "\\positionMotion.prm");
		@SuppressWarnings("unchecked")
		ArrayList<Point2d> targetList = (ArrayList<Point2d>)FileUtil.readObject(folder + "\\targetList.dat");
		getModule(ItemListModule.class).addSingleItem("Motion", totalMotion);
		TimeBasedList<Point3d> t3dList= new TimeBasedList<Point3d>();
		for (Point2d s : targetList){
			t3dList.add(Pose2d.to3d(s));
		}
		getModule(ItemListModule.class).addSingleItem("Target", t3dList, new ItemDescription(new Vector3d(0, 1, 0)));
		maxFrame = totalMotion.size()-1;
	}
	
	private void presetLocomotion2(String folder){
		canvas.eye = new Vector3d(377.98403561258516, 239.929273150635, 69.75355259425797);
		canvas.center = new Vector3d(68.06177369785054, 15.871544909848982, 13.64084707505449);
		canvas.upVector = new Vector3d(-0.584008030446584, 0.8092135323237658, -0.06409430144710053);
		
		PositionResultMotion totalMotion = (PositionResultMotion)FileUtil.readObject(folder + "\\positionMotion.prm");
		@SuppressWarnings("unchecked")
		ArrayList<DPStatus> statusList = (ArrayList<DPStatus>)FileUtil.readObject(folder + "\\statusList.dat");
		
		getModule(ItemListModule.class).addSingleItem("Motion", totalMotion);
		
		TimeBasedList<Point3d> targetList= new TimeBasedList<Point3d>();
		for (DPStatus s : statusList){
			targetList.add(Pose2d.to3d(s.targetPoint));
		}
		ItemDescription d1 = new ItemDescription(new Vector3d(0, 1, 0));
		d1.size = 8;
		getModule(ItemListModule.class).addSingleItem("Target", targetList, d1);
		
		TimeBasedList<Pose2d> dirList= new TimeBasedList<Pose2d>();
		for (int i = 0; i < statusList.size(); i++) {
			if (statusList.get(i).useDirection){
				dirList.add(statusList.get(i).direction);
			} else {
				dirList.add(null);
			}
		}
		ItemDescription d2 = new ItemDescription(new Vector3d(1, 1, 0));
		d2.size = 10;
		getModule(ItemListModule.class).addSingleItem("Direction", dirList, d2);
		
		maxFrame = totalMotion.size()-1;
	}
	
	private class Replay extends OpenGLReplay{
		@Override
		protected SWTViewableCanvas getCanvas() {
			return canvas;
		}

		@Override
		protected boolean update(int frame) {
			if (frame >= maxFrame) return true;
			getModule(MainViewerModule.class).setTimeIndex(frame);
			return false;
		}
	}

	public static void main(String[] args) {
		MainApplication.run(new OpenGLReplayModule());
	}
}
