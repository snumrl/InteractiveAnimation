package mrl.motion.neural.basket.sketch;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mrl.util.MathUtil;

public class SketchLine {
	
//	public static final int TYPE_DRIBBLE = 0;
//	public static final int TYPE_PASS = 1;
//	public static final int TYPE_SCREEN = 2;
//	public static final int TYPE_SHOOT = 3;
//	public static final String[] TYPE_LIST = {
//		"Dribble", "Pass", "Screen", "Shoot"
//	};
	
	public enum LineType{
		Dribble, Pass, Screen, Run, Feint, 
	}

	public SketchNode source;
	public SketchNode target;
	ArrayList<Point2d> points = new ArrayList<Point2d>();
	Vector2d lineVector;
	int type;
	
	// -1 left 1 right
	public int direction;
	
	public void addPoint(Point2d p){
		if (type() == LineType.Pass && points.size() >= 2){
			points.set(1, p);
		} else if(type() == LineType.Feint && points.size() == 1) {
			Vector2d v1 = MathUtil.sub(p,  points.get(0));
			if(v1.length() < SketchCanvas.NODE_RADIUS + 50) return;
			
			Vector2d basketDir = MathUtil.sub(SketchCanvas.BASKET_POS, points.get(0));
			basketDir.normalize();
			Vector2d leftDir = new Vector2d(basketDir.y, -basketDir.x);
			
			Point2d origin = MathUtil.add(points.get(0), MathUtil.scale(basketDir, SketchCanvas.NODE_RADIUS)); 
			
			points.add(origin);			
			if(MathUtil.cross(v1, basketDir) < 0) {
				direction = -1;
				Point2d p0 = new Point2d(origin);
				p0.add(MathUtil.scale(basketDir, 6));
				p0.add(MathUtil.scale(leftDir, -20));
				points.add(p0);
				
				Point2d p1 = new Point2d(p0);
				p1.add(MathUtil.scale(basketDir, 12));
				p1.add(MathUtil.scale(leftDir, 40));
				points.add(p1);
				
				Point2d p2 = new Point2d(p1);
				p2.add(MathUtil.scale(basketDir, 6));
				p2.add(MathUtil.scale(leftDir, -20));
				points.add(p2);				
			}
			else {
				direction = 1;
				Point2d p0 = new Point2d(origin);
				p0.add(MathUtil.scale(basketDir, 6));
				p0.add(MathUtil.scale(leftDir, 20));
				points.add(p0);				
				
				Point2d p1 = new Point2d(p0);
				p1.add(MathUtil.scale(basketDir, 12));
				p1.add(MathUtil.scale(leftDir, -40));
				points.add(p1);
				
				Point2d p2 = new Point2d(p1);
				p2.add(MathUtil.scale(basketDir, 6));
				p2.add(MathUtil.scale(leftDir, 20));
				points.add(p2);
			}
			/*
			Vector2d v1 = MathUtil.sub(points.get(1), points.get(0));
			Vector2d v2 = MathUtil.sub(p, points.get(1));

			v1.normalize();
			v2.normalize();
			
			if(v1.dot(v2) < -0.2) return;
			*/
			points.add(p);
		}
		else {
			points.add(p);
		}
	}
	
	public LineType type(){
		return LineType.values()[type];
	}
	
	public ArrayList<Point2d> getPoints() {
		if (target == null) return points;
		
		Vector2d vector = MathUtil.sub(target.position, source.position);
		double angle1 = Math.atan2(vector.y, vector.x);
		
		double angle2 = Math.atan2(lineVector.y, lineVector.x);
		double angleDiff = angle1 - angle2;
		double sin = Math.sin(angleDiff);
		double cos = Math.cos(angleDiff);
		double scale = vector.length()/lineVector.length();
		
		double rotAngle = Math.toRadians(20);
		double offset = 1;
		if (type() == LineType.Screen){
			rotAngle = Math.toRadians(90);
			offset = 10;
		}
		
		ArrayList<Point2d> list = new ArrayList<Point2d>();
		for (Point2d point : points){
			Vector2d v = new Vector2d(point);
			v.set(cos*v.x - sin*v.y, sin*v.x + cos*v.y);
			v.scale(scale);
			Point2d p = new Point2d();
			p.add(source.position, v);
			
			if (p.distance(target.position) < SketchCanvas.NODE_RADIUS + offset){
				break;
			}
			
			list.add(p);
		}
		
		if (list.size() == 0) return list;
		
		Point2d p1 = list.get(list.size() - 1);
		if (p1.distance(target.position) > SketchCanvas.NODE_RADIUS + offset){
			double distance = p1.distance(target.position);
			Point2d p = new Point2d();
			p.interpolate(target.position, p1, (SketchCanvas.NODE_RADIUS + offset)/distance);
			list.add(p);
			p1 = p;
		}
		
		int p2Offset = 1;
		Point2d p2 = null;
		while (true){
			p2 = list.get(Math.max(0, list.size() - p2Offset));
			if (p2.distance(p1) >= SketchCanvas.NODE_RADIUS*0.9) break;
			if (list.size() - p2Offset <= 0) break;
			p2Offset++;
		}
		Vector2d v = MathUtil.sub(p2, target.position);
		v.normalize();
		v.scale(SketchCanvas.NODE_RADIUS);
		
		
		Vector2d v1 = MathUtil.rotate(v, rotAngle);
		Vector2d v2 = MathUtil.rotate(v, -rotAngle);
		Point2d arrow1 = new Point2d();
		arrow1.add(p1, v1);
		Point2d arrow2 = new Point2d();
		arrow2.add(p1, v2);
		list.add(arrow1);
		list.add(p1);
		list.add(arrow2);
		
		return list;
	}
	
	public void fix(){
		lineVector = MathUtil.sub(target.position, source.position);
		
		for (Point2d p : points){
			p.sub(source.position);
		}
	}
	
}
