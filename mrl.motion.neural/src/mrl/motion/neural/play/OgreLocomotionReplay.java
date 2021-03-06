package mrl.motion.neural.play;

import static mrl.motion.neural.play.OgreRecorder.BALL_FILE;
import static mrl.motion.neural.play.OgreRecorder.STATUS_FILE;
import static mrl.motion.neural.play.OgreRecorder.motionFile;

import java.io.File;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.neural.data.PointIKSolver_CMU;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.Configuration;
import mrl.util.FileUtil;
import mrl.util.Utils;

public class OgreLocomotionReplay {
	
	public static int MAX_LENGTH = -1;
	public static String capturePath = null;
	public static int frameOffset = 0;
	public static boolean fixCamera = true;
	
	public ArrayList<MotionData> mDataList;
	public ArrayList<Point3d> ballTrajectory;
	public ArrayList<Point3d> targetTrajectory;
	public ArrayList<OgreStatus> statusList;
	
	private static int clipStart = 0;

	public OgreLocomotionReplay(String name){
		MotionData mData = new BVHParser().parse(new File(Configuration.BASE_MOTION_FILE));
		PointIKSolver_CMU ikSolver = new PointIKSolver_CMU(mData.skeletonData, mData.motionList.get(0));
		
		String folder = OgreRecorder.OUTPUT_FOLDER + "\\" + name;
		
		PositionResultMotion totalMotion = (PositionResultMotion)FileUtil.readObject(folder + "\\positionMotion.prm");
		@SuppressWarnings("unchecked")
		ArrayList<Point2d> targetList = (ArrayList<Point2d>)FileUtil.readObject(folder + "\\targetList.dat");
		
		mDataList = new ArrayList<MotionData>();
		mDataList.add(ikSolver.solve(totalMotion));
		ballTrajectory = new ArrayList<Point3d>();
		statusList = new ArrayList<OgreStatus>();
		targetTrajectory = new ArrayList<Point3d>();
		
		for (int i = 0; i < targetList.size(); i++) {
			OgreStatus status = new OgreStatus();
			status.setNull();
			statusList.add(status);
			ballTrajectory.add(null);
			targetTrajectory.add(Pose2d.to3d(targetList.get(i)));
		}
		System.out.println("sizeeee :: " + targetList.size());
	}
	
	
	public void run(double[] ogreParams){
		OgreJNI.open(ogreParams);
		
		OgreJNI.setMotion(Utils.toArray(mDataList));
		OgreJNI.setBall(ballTrajectory);
		if (targetTrajectory.size() > 0) OgreJNI.setBall2(targetTrajectory);
		OgreStatus fixedCamera = null;
		
		int prevKey = -1;
		long startTime = -1;
		int frame = -1;
		while (true){
			if (OgreJNI.isClosed()) break;
			OgreStatus status = OgreJNI.getStatus();
			
			int key = status.key;
			if (key == prevKey){
				key = -1;
			} else {
				prevKey = key;
			}
			
			if (key == (int)'s'){
				startTime = System.currentTimeMillis();
				frame = 0;
				if (fixCamera){
					fixedCamera = status;
				}
			} else if (key == (int)'q'){
				break;
			}
			
			if (startTime >= 0){
				while (frame < statusList.size()){
					int dt = (int)(System.currentTimeMillis() - startTime);
					int tIndex = dt/33;
					if (frame > tIndex) break;
					
					if (capturePath != null && !OgreJNI.isCaptured()) break;
					
					
					int f = frameOffset + frame + clipStart;
					if (f >= statusList.size()){
						startTime = -1;
						System.out.println("generate finished.");
						break;
					}
					OgreStatus s = statusList.get(f);
					if (fixedCamera != null){
						s.setCamera(fixedCamera);
					}
					OgreJNI.setStatus(s.toData(f));
					if (capturePath != null && f > 0){
						OgreJNI.doCapture(capturePath + "_" + String.format("%04d", f - clipStart) + ".png");
					}
					frame++;
					System.out.println("frame :: " + frame + " / " + statusList.size());
				}
				if (frame >= statusList.size()){
					startTime = -1;
					System.out.println("generate finished.");
				}
			}
			
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
		}
		OgreJNI.close();
	}
	
	private static String filePath(int frame){
		return capturePath + "_" + String.format("%04d", frame) + ".png";
	}
	
	static void setCapture(String name){
		String folder = "D:\\data\\RNN\\output\\" + name;
		new File(folder).mkdirs();
		capturePath = folder + "\\capture";
		for (int i = 1; i < 10000; i++) {
			if (new File(filePath(i)).exists()){
				frameOffset = i;
			} else {
				break;
			}
		}
		System.out.println("start frame :: " + frameOffset);
	}
	
	public static void main(String[] args) {
		String name;
		name = "loco_indian2";
		setCapture(name);
//		clipStart = 390;
		Configuration.BASE_MOTION_FILE = "cmu.bvh";
		MotionDataConverter.setCMUAllJoints();
		double[] params = new double[]{ 0 };
		OgreJNI.setUseCMUMotion();
		OgreLocomotionReplay replay;
		replay = new OgreLocomotionReplay(name);
//		replay = new OgreReplay("dribble_new_nv");
		replay.run(params);
	}
}
