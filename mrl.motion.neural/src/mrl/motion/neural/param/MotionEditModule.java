package mrl.motion.neural.param;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.position.PositionMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.MainViewerModule.MainViewer;
import mrl.motion.viewer.module.MotionListModule;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.PointListViewer2D;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.ListViewerModule;
import mrl.widget.app.ListViewerModule.ListListener;
import mrl.widget.app.Module;
import mrl.widget.app.Point2dViewerModule;

import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;

public class MotionEditModule extends Module{
	
	private MDatabase database;
	private ArrayList<Motion> motionList;
	private HashMap<Point2d, Integer> indexMap = new HashMap<Point2d, Integer>();

	@Override
	protected void initializeImpl() {
//		getModule(MotionListModule.class).loadMotionFolder("indian_all");
		BVHParser.MAX_FRAME = 300;
		database = RNNDataGenerator.loadCMUDatabase("walk");
//		database = RNNDataGenerator.loadCMUDatabase("indian");
		
		getModule(MotionListModule.class).getTree().addListener(SWT.MouseDoubleClick, new Listener() {
			@Override
			public void handleEvent(Event event) {
				MotionData[] mData = getModule(MotionListModule.class).getSelectedMotionData();
				if (mData == null) return;
				motionList = mData[0].motionList;
				
				ArrayList<Point2d> pointList = new ArrayList<Point2d>();
				int tMargin = 16;
				indexMap = new HashMap<Point2d, Integer>();
				for (int i = 0; i < motionList.size(); i++) {
					Point2d p = new Point2d(Pose2d.to2d(MathUtil.getTranslation(motionList.get(i).root())));
					pointList.add(p);
					indexMap.put(p, i);
				}
//				for (int i = 0; i < motionList.size() - tMargin; i++) {
//					Pose2d p1 = PositionMotion.getPose(motionList.get(i));
//					Pose2d p2 = PositionMotion.getPose(motionList.get(i + tMargin));
//					Pose2d pose = Pose2d.relativePose(p1, p2);
//					Point2d p = new Point2d(pose.position);
//					pointList.add(p);
//					indexMap.put(p, i);
//				}
				getModule(Point2dViewerModule.class).setPointList(pointList);
			}
		});
		
		{
			motionList = generateGraphMotion();
			
			ArrayList<Point2d> pointList = new ArrayList<Point2d>();
			int tMargin = 16;
			indexMap = new HashMap<Point2d, Integer>();
			for (int i = 0; i < motionList.size() - tMargin; i++) {
				Pose2d p1 = PositionMotion.getPose(motionList.get(i));
				Pose2d p2 = PositionMotion.getPose(motionList.get(i + tMargin));
				Pose2d pose = Pose2d.relativePose(p1, p2);
				Point2d p = new Point2d(pose.position);
				pointList.add(p);
				indexMap.put(p, i);
			}
			getModule(Point2dViewerModule.class).setPointList(pointList);
			getModule(ItemListModule.class).addSingleItem("generated", new MotionData(motionList));
		}
		
		
		PointListViewer2D pViewer = getModule(Point2dViewerModule.class).getViewer();
		pViewer.addListener(SWT.Selection, new Listener() {
			@Override
			public void handleEvent(Event event) {
				Point2d p = pViewer.getSelectedPoint();
				if (p == null) return;
				Integer index = indexMap.get(p);
				if (index == null) return;
				
				getModule(MainViewerModule.class).setTimeIndex(index);
				MainViewer viewer = getModule(MainViewerModule.class).getMainViewer();
				Vector3d t = MathUtil.getTranslation(motionList.get(index).root());
				t.scale(viewer.getScale());
				System.out.println(t);
				t.sub(viewer.center);
				t.y = 0;
				viewer.center.add(t);
				viewer.eye.add(t);
				System.out.println(viewer.center);
				System.out.println("############");
			}
		});
		
		addMenu("&Menu", "&Test\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				int index = getModule(MainViewerModule.class).getTimeIndex();
				System.out.println("iiii : " + index + " : " + motionList.get(index));
				int margin = 15;
				for (int i = -margin; i <= margin; i++) {
					int idx = index + i;
					System.out.println(i + " :: " + motionList.get(idx));
				}
			}
		});
		
//		getModule(MainViewerModule.class);
//		MotionEditParam param = new MotionEditParam();
//		LocomotionSTM stm = new LocomotionSTM(param);
//		ArrayList<double[]> sequence = stm.generate(10);
//		PositionResultMotion motion = MotionPrediction.getMotion(sequence);
//		getModule(ItemListModule.class).addSingleItem("motion", motion);
	}
	
	private ArrayList<Motion> generateGraphMotion(){
		MotionParamGraph param = new MotionParamGraph(database);
//		getModule(ListViewerModule.class).setItems(param.getNonSequentialLinks(), new ListListener<MPLink>(){
//			@Override
//			public String[] getColumnHeaders() {
//				return new String[]{ "x" , "y", "distance" };
//			}
//			@Override
//			public String[] getTableValues(MPLink item) {
//				return Utils.toStringArrays(item.source().motionIndex, item.target().motionIndex, item.distance());
//			}
//			@Override
//			public void onItemSelection(MPLink item) {
//				int x = item.source().motionIndex;
//				int y = item.target().motionIndex;
//				int margin = 30;
//				MotionSegment s1 = new MotionSegment(param.mList, x - margin, x-1);
//				MotionSegment s2 = new MotionSegment(param.mList, y, y + margin);
//				MotionSegment s3 = new MotionSegment(param.mList, x - margin, x + margin);
//				s1 = MotionSegment.stitch(s1, s2, true);
//				getModule(ItemListModule.class).addSingleItem("motion1", new MotionData(s1.getMotionList()));
//				getModule(ItemListModule.class).addSingleItem("motion2", new MotionData(s3.getMotionList()));
//				getModule(MainViewerModule.class).replay();
//			}
//		});
		
		ParamGraphExplorer exp = new ParamGraphExplorer(param);
		ArrayList<int[]> segments = exp.explore(50000);
//		ArrayList<int[]> segments = exp.explore(param.nodeList().size()*1);
		System.out.println(Arrays.toString(MathUtil.getStatistics(MathUtil.toDouble(exp.visitCounts))));
		MotionSegment s = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		ArrayList<Motion> mList = new MotionSegment(s).getMotionList();
		Matrix4d p = Pose2d.globalTransform(PositionMotion.getPose(mList.get(0)), Pose2d.BASE).to3d();
		for (Motion m : mList){
			m.root().mul(p, m.root());
		}
		
		getModule(ItemListModule.class).addSingleItem("original", new MotionData(mList));
		TrajectoryEditParam tEdit = new TrajectoryEditParam(); 
		s = tEdit.edit(s);
		FootSlipCleanup.clean(s);
//		getModule(ListViewerModule.class).setItems(tEdit.editedList, new ListListener<MotionSegment>() {
//			@Override
//			public String[] getColumnHeaders() {
//				return new String[] { "c" };
//			}
//
//			@Override
//			public String[] getTableValues(MotionSegment item) {
//				return Utils.toStringArrays(item.length());
//			}
//
//			@Override
//			public void onItemSelection(MotionSegment item) {
//				getModule(ItemListModule.class).addSingleItem("segment", new MotionData(item.getEntireMotion()));
//				getModule(MainViewerModule.class).replay();
//			}
//		});
		
		return s.getMotionList();
	}
}
