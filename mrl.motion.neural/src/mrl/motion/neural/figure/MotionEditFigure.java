package mrl.motion.neural.figure;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3i;
import javax.vecmath.Vector2d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.position.PositionMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.MathUtil;
import mrl.util.Matrix2d;
import mrl.util.Utils;
import mrl.widget.Rectangle2d;
import mrl.widget.ScalableCanvas2D;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;
import mrl.widget.app.MainApplication.WindowPosition;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.graphics.Transform;
import org.eclipse.swt.widgets.Composite;

public class MotionEditFigure extends ScalableCanvas2D{
	
	public TrajectoryEditParam edit;
	public ArrayList<MotionSegment> originList;
	public ArrayList<MotionSegment> editedList;
	
	private GC gc;
	private HashMap<Point3i, Color> colorMap = new HashMap<Point3i, Color>();
	
	private int C_RADIUS = 22;
	private int T_INTERVAL = 9;
//	private int C_RADIUS = 16;
//	private int T_INTERVAL = 6;
	

	public MotionEditFigure(Composite parent, TrajectoryEditParam edit) {
		super(parent);
		this.edit = edit;
		originList = edit.originList;
		editedList = edit.editedList;
	}
	
	private Color getColor(int r, int g, int b){
		Point3i p = new Point3i(r, g, b);
		Color c = colorMap.get(p);
		if (c == null){
			c = new Color(getDisplay(), r, g, b);
			colorMap.put(p, c);
		}
		return c;
	}

	@Override
	protected void drawContents(GC gc) {
		this.gc = gc;
		
		MotionSegment current = new MotionSegment(originList.get(0), 0, 0);
		Matrix4d t = new Matrix4d();
		t.rotY(-Math.PI/2);
		for (Motion m : current.getEntireMotion()){
			m.root().mul(t, m.root());
		}
		current.updateNotBlendedAsCurrent();
		
		
		gc.setAntialias(SWT.ON);
		gc.setAdvanced(true);
		
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
		
		
		{
			Point2d source = pos(oList.get(0).firstMotion());
			Point2d target = pos(oList.get(0).getMotionList().get(lenList.get(0) - 1));
			gc.setAlpha(50);
			gc.setBackground(getColor(255, 128, 0));
//			gc.setBackground(getColor(0, 128, 255));
			double len = source.distance(target);
			int angle = r(Math.toDegrees(MathUtil.directionalAngle(new Vector2d(1, 0),  MathUtil.sub(target, source))));
			gc.fillArc(r(source.x - len), r(source.y - len), r(len*2), r(len*2), -angle-45, 90);
			gc.setBackground(getColor(255, 255, 255));
			gc.setAlpha(255);
			len = len*0.7;
			gc.fillOval(r(source.x - len), r(source.y - len), r(len*2), r(len*2));
			
			gc.setAlpha(50);
			Vector2d v = MathUtil.sub(target, source);
			Point2d t1 = MathUtil.add(source, MathUtil.rotate(v, Math.toRadians(45)));
			Point2d t2 = MathUtil.add(source, MathUtil.rotate(v, Math.toRadians(-45)));
			gc.setLineDash(new int[]{ 3, 1 });
			gc.drawLine(r(source.x), r(source.y), r(t1.x), r(t1.y));
			gc.drawLine(r(source.x), r(source.y), r(t2.x), r(t2.y));
			gc.setAlpha(255);
			gc.setLineDash(null);
		}
		
		
		for (int i = 0; i < oList.size(); i++) {
			gc.setAlpha(50);
			gc.setBackground(getColor(0, 0, 255));
			drawTrajectory(oList.get(i));
			gc.setAlpha(255);
			gc.setBackground(getColor(255, 0, 0));
			drawTrajectory(eList.get(i));
		}
		
		gc.setBackground(getColor(128, 128, 255));
		for (int i = 0; i < oList.size(); i++) {
			drawPose(oList.get(i).getMotionList().get(lenList.get(i) - 1));
//			drawPose(oList.get(i).getEntireMotion().get(MotionSegment.BLEND_MARGIN() + lenList.get(i)));
		}
		gc.setBackground(getColor(0, 255, 0));
		for (int i = 0; i < eList.size(); i++) {
			drawPose(eList.get(i).firstMotion());
		}
		drawPose(eList.get(eList.size()-1).lastMotion());
		
		
		
		ArrayList<Motion> origin = edit.origin.getMotionList();
		ArrayList<Motion> edited = edit.edited.getMotionList();
		MotionData.adjustKnot(new MotionData[]{ new MotionData(edited) });
		double maxKnot = Math.max(Utils.last(origin).knot, Utils.last(edited).knot);
		
		int yOffset = 50;
		gc.setLineWidth(2);
		gc.setAlpha(50);
		gc.setBackground(getColor(255, 128, 0));
		
		double w = origin.get(lenList.get(0)).knot;
		
		drawTimeBoundary(w*0.8, w*1.2, yOffset, maxKnot);
		gc.setAlpha(255);
		
		
		drawTimeBar(edit.origin.getMotionList(), 0, maxKnot);
		drawTimeBar(edit.edited.getMotionList(), yOffset, maxKnot);
		
		double knotOffset = 0;
		gc.setBackground(getColor(128, 128, 255));
		int idx = 0;
		for (int i = -1; i < oList.size(); i++) {
			if (i < 0){
				idx = 0;
			} else {
				idx += lenList.get(i);
			}
			System.out.println(i + " :: " + idx);
			drawTimePoint(origin.get(idx), knotOffset, 0, maxKnot);
		}
		
		gc.setBackground(getColor(0, 255, 0));
		for (int i = -1; i < eList.size(); i++) {
			if (i < 0){
				idx = 0;
			} else {
				idx += lenList.get(i);
			}
			drawTimePoint(edited.get(idx), knotOffset, yOffset, maxKnot);
		}
		gc.setLineWidth(1);
//		drawTimePoint(eList.get(eList.size()-1).lastMotion(), knotOffset, yOffset, maxKnot);
	}
	
	private void drawTimeBoundary(double kStart, double kEnd, int yOffset, double maxKnot){
		int baseY = -1500;
		int baseX = 0;
		int width = 1000;
		int height = 10;
		
		int margin = 10;
		
		double pos = kStart/maxKnot;
		int x1 = r(baseX + width*pos);
		pos = kEnd/maxKnot;
		int x2 = r(baseX + width*pos);
		
		gc.fillRectangle(x1, baseY - height - margin, x2 - x1, margin*2 + height*2 + yOffset);
	}
	private void drawTimeBar(ArrayList<Motion> mList, int yOffset, double maxKnot){
		int baseY = -1500;
		int baseX = 0;
		int width = 1000;
		int height = 10;
		int y = baseY + yOffset;
		int lastX = -1;
		for (int i = 0; i < mList.size() - T_INTERVAL; i+=T_INTERVAL) {
			Motion motion = mList.get(i);
			double pos = motion.knot/maxKnot;
			int x = r(baseX + width*pos);
			drawLine(gc, x, y - height, x, y + height);
			lastX = x;
		}
		drawLine(gc, baseX, y, lastX, y);
	}
	
	private void drawTimePoint(Motion motion, double knotOffset, int yOffset, double maxKnot){
		int baseY = -1500;
		int baseX = 0;
		int width = 1000;
		int radius = 10;
		int y = baseY + yOffset;
		
		double pos = (motion.knot - knotOffset)/maxKnot;
		int x = r(baseX + width*pos);
		fillOval(gc, x - radius, y - radius, radius*2, radius*2);
	}
	
	private int r(double v){
		return (int)Math.round(v);
	}
	
	private Point2d[] rectangle(double width, double height, double tx, double ty){
		Point2d[] points = {
			new Point2d(0, 0),
			new Point2d(width, 0),
			new Point2d(width, height),
			new Point2d(0, height),
		};
		for (Point2d p : points){
			p.x += tx - width/2;
			p.y += ty - height/2;
		}
		return points;
	}
	
	private void fillPolygon(Matrix2d m, Point2d[] points){
		int[] values = new int[points.length*2];
		for (int i = 0; i < points.length; i++) {
			Point2d p = new Point2d(points[i]);
			m.transform(p);
			values[i*2 + 0] = (int)p.x;
			values[i*2 + 1] = (int)p.y;
		}
		gc.fillPolygon(values);
	}
	
	private void drawPose(Motion motion){
		Pose2d p = PositionMotion.getPose(motion);
		Matrix2d m = Pose2d.globalTransform(Pose2d.BASE, p);
		fillPolygon(m, rectangle(60, 20, 20, 0));
		fillPolygon(m, rectangle(20, 80, 0, 0));
//		fillPolygon(m, rectangle(80, 20, 0, 0));
//		fillPolygon(m, rectangle(20, 60, 0, -20));
		
		int s = 12;
		drawOval(gc, p.position.x - s/2, p.position.y - s/2, s, s);
	}
	
//	private void drawRange(GC gc, Point2d p1, Point2d p2){
//		MathUtil.
//		gc.fillArc(x, y, width, height, startAngle, arcAngle);
//	}
	
	private MotionSegment merged(ArrayList<MotionSegment> list, int start){
		MotionSegment merged = null;
		for (int i = start; i < list.size()-1; i++) {
			merged = MotionSegment.stitch(merged, new MotionSegment(list.get(i)), true);
		}
		return merged;
	}
	
	private Motion lastAfterMotion(MotionSegment segment){
		return segment.getEntireMotion().get(segment.length() + MotionSegment.BLEND_MARGIN());
	}
	
	private Point2d pos(Motion motion){
		return new Point2d(Pose2d.to2d(MathUtil.getTranslation(motion.root())));
	}
	
	private void drawTrajectory(MotionSegment segment){
		ArrayList<Motion> mList = segment.getMotionList();
		mList.add(lastAfterMotion(segment));
		ArrayList<Point2d> pList = new ArrayList<Point2d>();
		mList.forEach(m -> pList.add(pos(m)));
		
		Point2d prevP = null;
		for (int i = 0; i < pList.size(); i+=T_INTERVAL) {
			Point2d p1 = pList.get(i);
			if (prevP != null){
				drawLine(gc, p1.x, p1.y, prevP.x, prevP.y);
			}
			prevP = p1;
		}
		
		for (int i = 0; i < pList.size(); i+=T_INTERVAL) {
			Point2d p = pList.get(i);
			double offset = C_RADIUS/2d;
			fillOval(gc, p.x-offset, p.y-offset, C_RADIUS, C_RADIUS);
		}
	}

	@Override
	protected boolean onMouseDown(Point2d p, MouseEvent e) {
		return false;
	}

	@Override
	protected boolean onMouseMove(Point2d p, MouseEvent e) {
		return false;
	}

	@Override
	protected boolean onMouseUp(Point2d p, MouseEvent e) {
		return false;
	}

	@Override
	protected boolean onMouseDoubleClick(Point2d p, MouseEvent e) {
		return false;
	}

	@Override
	protected boolean onMouseScrolled(Point2d p, MouseEvent e) {
		return false;
	}

	@Override
	protected Rectangle getContentBoundary() {
		return null;
	}

	@Override
	protected void onBoundarySelection(Rectangle2d boundary, MouseEvent e) {
	}
	
	static class FigureModule extends Module{
		
		private MotionEditFigure figure;

		@Override
		protected void initializeImpl() {
			MathUtil.random.setSeed(38);
			TrajectoryEditParam edit = new BasketDataGenerator().generateDribble();
			figure = addWindow(new MotionEditFigure(dummyParent(), edit), WindowPosition.Main);
			
			ArrayList<Motion> origin = edit.origin.getMotionList();
			ArrayList<Motion> edited = edit.edited.getMotionList();
			MotionData.adjustKnot(new MotionData[]{ new MotionData(edited) });
			for (int i = 0; i < origin.size(); i++) {
				System.out.println(Utils.toString(i, origin.get(i).knot, edited.get(i).knot));
			}
		}
	}
	
	public static void main(String[] args) {
//		{
////			MathUtil.random.setSeed(108);
//			MathUtil.random.setSeed(38);
//			BasketDataGenerator g = new BasketDataGenerator();
//			g.generateDribble();
//			MainViewerModule.run(new MotionData(g.segment.getMotionList()));
//			System.exit(0);
//		}
		MainApplication.run(new FigureModule());
	}

}
