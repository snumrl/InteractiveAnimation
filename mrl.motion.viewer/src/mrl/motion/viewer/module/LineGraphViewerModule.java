package mrl.motion.viewer.module;

import java.util.HashMap;

import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.position.PositionResultMotion;
import mrl.util.MathUtil;
import mrl.widget.ColorMapping;
import mrl.widget.WidgetUtil;
import mrl.widget.app.MainApplication.WindowPosition;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.PaintEvent;
import org.eclipse.swt.events.PaintListener;
import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Listener;

public class LineGraphViewerModule extends Module{
	
	private List list;
	private LineGraphViewer viewer;

	@Override
	protected void initializeImpl() {
		Composite container = addWindow(new Composite(dummyParent(), SWT.NONE), WindowPosition.Right);
		container.setLayout(new GridLayout(2, false));
		
		list = new List(container, SWT.MULTI);
		list.setLayoutData(WidgetUtil.gridData(false, true, 200, SWT.DEFAULT));
		
		viewer = new LineGraphViewer(container);
		viewer.setLayoutData(WidgetUtil.gridData(true, true));
		
		getModule(MainViewerModule.class).addTimeListener(new Listener() {
			@Override
			public void handleEvent(Event event) {
				viewer.timeIndex = getModule(MainViewerModule.class).getTimeIndex();
				viewer.canvas.redraw();
			}
		});
	}
	
	public void setMotionData(MotionData motionData){
		String[] jointList = SkeletonData.keyList;
		String[] labelList = new String[jointList.length*3];
		for (int i = 0; i < jointList.length; i++) {
			labelList[i*3 + 0] = jointList[i] + "_x";
			labelList[i*3 + 1] = jointList[i] + "_y";
			labelList[i*3 + 2] = jointList[i] + "_z";
		}
		double[][] dataList = new double[motionData.motionList.size()][labelList.length];
		for (int i = 0; i < dataList.length; i++) {
			HashMap<String, Point3d> posMap = Motion.getPointData(SkeletonData.instance, motionData.motionList.get(i));
			
			for (int j = 0; j < jointList.length; j++) {
				Point3d p = posMap.get(jointList[j]);
				dataList[i][j*3 + 0] = p.x; 
				dataList[i][j*3 + 1] = p.y; 
				dataList[i][j*3 + 2] = p.z; 
			}
		}
		viewer.setDataList(dataList, labelList);
		viewer.canvas.redraw();
	}
	
	public void setMotionData(PositionResultMotion motionData){
//		MotionDataConverter.
//		String[][] jointPairs = MotionDataConverter.jointPairs;
//				
//		for (String[] pair : jointPairs){
//			for (int i = 0; i < pair.length-1; i++) {
//				
//			}
//		}
//		String[] jointList = SkeletonData.keyList;
//		String[] labelList = new String[jointList.length*3];
//		for (int i = 0; i < jointList.length; i++) {
//			labelList[i*3 + 0] = jointList[i] + "_x";
//			labelList[i*3 + 1] = jointList[i] + "_y";
//			labelList[i*3 + 2] = jointList[i] + "_z";
//		}
//		double[][] dataList = new double[motionData.motionList.size()][labelList.length];
//		for (int i = 0; i < dataList.length; i++) {
//			HashMap<String, Point3d> posMap = Motion.getPointData(SkeletonData.instance, motionData.motionList.get(i));
//			
//			for (int j = 0; j < jointList.length; j++) {
//				Point3d p = posMap.get(jointList[j]);
//				dataList[i][j*3 + 0] = p.x; 
//				dataList[i][j*3 + 1] = p.y; 
//				dataList[i][j*3 + 2] = p.z; 
//			}
//		}
//		viewer.setDataList(dataList, labelList);
//		viewer.canvas.redraw();
	}
	
	
	public static class LineGraphViewer extends Composite implements PaintListener{
		
		private Canvas canvas;
		private String[] labelList;
		private double[][] dataList;
		private int timeIndex = -1;
		private ColorMapping cm;
		
		private double[] minMax;
		private double dWidth;

		public LineGraphViewer(Composite parent) {
			super(parent, SWT.NONE);
			this.setLayout(new FillLayout());
			canvas = new Canvas(this, SWT.DOUBLE_BUFFERED);
			cm = new ColorMapping(getDisplay());
		}
		
		public void setDataList(double[][] dataList, String[] labelList){
			this.dataList = dataList;
			this.labelList = labelList;
			minMax = MathUtil.getMinMax(dataList);
			dWidth = minMax[1] - minMax[0];
//			MathUtil.getStatistics(values)
		}

		@Override
		public void paintControl(PaintEvent e) {
			if (dataList == null || timeIndex < 0) return;
			
			Point size = canvas.getSize();
			
			GC gc = e.gc;
			gc.setBackground(getDisplay().getSystemColor(SWT.COLOR_WHITE));
			gc.fillRectangle(0, 0, size.x, size.y);
			
			int width = 800;
			int height = 300;
			
			int baseX = 50;
			int baseY = height+100;
			
			gc.setLineWidth(1);
			gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_BLACK));
			gc.drawLine(baseX, baseY, baseX, baseY - height);
			gc.drawLine(baseX, baseY, baseX + width, baseY);
			
			int count = 40;
			for (int i = -count; i <= count; i++) {
				int idx = timeIndex + i;
				if (idx < 0 || idx >= dataList.length - 1) continue;
				int idx2 = idx + 1;
				for (int dIdx = 0; dIdx < dataList[idx].length; dIdx++) {
					gc.setForeground(cm.getColor(dIdx));
					double v1 = dataList[idx][dIdx];
					double v2 = dataList[idx2][dIdx];
					int x1 = (count + i)*width/(count*2);
					int x2 = (count + i + 1)*width/(count*2);
					double y1 = baseY - ((v1 - minMax[0]) / dWidth)*height;
					double y2 = baseY - ((v2 - minMax[0]) / dWidth)*height;
					gc.drawLine(x1, MathUtil.round(y1), x2, MathUtil.round(y2));
				}
			}
			
			for (int i = 0; i < labelList.length; i++) {
				gc.setForeground(cm.getColor(i));
				gc.drawString(labelList[i], baseX + 10, 10 + i*30);
			}
			
			gc.setLineWidth(3);
			gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_GREEN));
			int midX = baseX + width/2;
			gc.drawLine(midX, baseY, midX, baseY - height);
		}
	}

}
