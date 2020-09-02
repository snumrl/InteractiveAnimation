package mrl.motion.neural.tennis;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.util.MathUtil;

public class TennisRacketData {

	public MotionData motionData;
	public ArrayList<TennisRacket> racketList;
	
	private static double RWidth = 30;
	private static double RHeight = 20;
	private static double RLen = 68.58 - RWidth;
	public static double[][] RacketPoints = {
		{ 0, 0, 0 },
		{ -(RLen), 0, 0 },
		{ -(RLen), RHeight/2, 0 },
		{ -(RLen + RWidth), RHeight/2, 0 },
		{ -(RLen + RWidth), -RHeight/2, 0 },
		{ -(RLen), -RHeight/2, 0 },
		{ -(RLen), 0, 0 },
	};
	
	public TennisRacketData(MotionData motionData) {
		this.motionData = motionData;
		
		racketList = new ArrayList<TennisRacketData.TennisRacket>();
		for (Motion motion : motionData.motionList){
			HashMap<String, Matrix4d> transforms = Motion.getTransformData(motionData.skeletonData, motion);
			Matrix4d matrix = transforms.get("RightHand");
			Vector3d translation = motionData.skeletonData.get("RightHand_End").transition;
			System.out.println("tt : " + translation);
			Vector3d base = new Vector3d(-1, 0, 0);
			Vector3d cross = MathUtil.cross(base, translation);
			cross.normalize();
			Matrix4d m = new Matrix4d();
			m.set(new AxisAngle4d(cross, base.angle(translation)));
			m.mul(matrix, m);
//			m = matrix;
			racketList.add(new TennisRacket(m));
		}
	}
	
	public static class TennisRacket{
		public Matrix4d matrix;
		public Point3d[] points;
		
		public TennisRacket(Matrix4d matrix) {
			this.matrix = matrix;
			points = new Point3d[RacketPoints.length];
			for (int i = 0; i < RacketPoints.length; i++) {
				Point3d p = new Point3d(RacketPoints[i]);
				matrix.transform(p);
				points[i] = p;
			}
		}
	}
}
