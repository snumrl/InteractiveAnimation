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
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.FileUtil;
import mrl.util.Utils;

public class OgreReplay {
	
	public static int MAX_LENGTH = -1;
	public static String capturePath = null;
	public static int frameOffset = 0;
	public static boolean fixCamera = true;
	
	public ArrayList<MotionData> mDataList;
	public ArrayList<Point3d> ballTrajectory;
	public ArrayList<Point3d> targetTrajectory;
	public ArrayList<OgreStatus> statusList;

	static int startOffset = 0;
	public OgreReplay(String name){
		String folder = OgreRecorder.OUTPUT_FOLDER + "\\" + name;
		mDataList = new ArrayList<MotionData>();
		for (int i = 0; i < 10; i++) {
			File file = new File(folder + "\\" + motionFile(i));
			if (!file.exists()) break;
			mDataList.add(new BVHParser().parse(file));
		}
		ballTrajectory = new ArrayList<Point3d>();
		for (double[] data : FileUtil.readDoubleFromString(folder + "\\"+ BALL_FILE)){
			Point3d p = new Point3d(data[0], data[1], data[2]);
			ballTrajectory.add(p);
		}
		statusList = new ArrayList<OgreStatus>();
		targetTrajectory = new ArrayList<Point3d>();
		String sFile = folder + "\\"+ STATUS_FILE;
		if (new File(sFile).exists()){
			for (double[] data : FileUtil.readDoubleFromString(sFile)){
				OgreStatus status = new OgreStatus();
				status.load(data);
				statusList.add(status);
				status.cursor = new Point2d(-100000, -100000);
				targetTrajectory.add(Pose2d.to3d(status.target));
			}
		} else {
			for (int i = 0; i < ballTrajectory.size(); i++) {
				OgreStatus status = new OgreStatus();
				status.setNull();
				statusList.add(status);
			}
		}
		if (MAX_LENGTH > 0){
			statusList = Utils.cut(statusList, 0, Math.min(MAX_LENGTH, statusList.size())-1);
		}
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
					
					
					int f = frameOffset + frame + startOffset;
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
						OgreJNI.doCapture(capturePath + "_" + String.format("%04d", f-startOffset) + ".png");
					}
					frame++;
					System.out.println("frame :: " + (frame+startOffset) + " / " + statusList.size());
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
//		name = "game3";
//		setCapture(name);
		
		
//		name = "game_lc";
//		name = "shoot_o_lc";
//		name = "sketch_full";
//		name = "giveAndGo";
//		double ratio = 3.35;
//		double offset = 948.71*ratio/3.7;
//		double[] params = { 1, 0, ratio, offset, 0.25*0.8, 1 };
		
//		name = "shoot_o";
//		name = "dribble_new7";
		
//		MAX_LENGTH = 720;
//		name = "dribble_no_ball";
		
		startOffset = 353;
		name = "drb_e128k";
//		startOffset = 240;
//		name = "drb_e4k";
//		startOffset = 185;
//		name = "drb_origin";
		
//		name = "dribble_all";
//		name = "sketch2";
//		name = "walk_new_small";
		double[] params = new double[]{ 0 };
//		params = OgreJNI.courtParam();
		setCapture(name);
		
		OgreReplay replay;
		replay = new OgreReplay(name);
//		replay = new OgreReplay("dribble_new_nv");
		replay.run(params);
	}
}
