package mrl.motion.neural.play;

import java.io.File;
import java.util.ArrayList;

import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHWriter;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.Configuration;
import mrl.util.FileUtil;
import mrl.util.Pair;
import mrl.util.Utils;

public class OgreRecorder {
	
	public static String OUTPUT_FOLDER = "output";
	public static String BALL_FILE = "ball.txt";
	public static String STATUS_FILE = "status.txt";

	private RuntimeController controller;

	public OgreRecorder(RuntimeController controller) {
		this.controller = controller;
	}
	
	
	public void run(double[] ogreParams){
		OgreJNI.open(ogreParams);
		
		ArrayList<RecordData> recorded = new ArrayList<RecordData>();
		try{
			controller.init();
			System.out.println("Init Finished.");
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
				} else if (key == (int)'q'){
					OgreJNI.close();
					return;
				} else if (key == (int)'c'){
					break;
				}
				if (startTime >= 0){
					while (true){
						int dt = (int)(System.currentTimeMillis() - startTime);
						int tIndex = dt/33;
						if (frame > tIndex) break;
						
						Pair<Motion[], Point3d> result = controller.process(frame, Pose2d.to2d(status.mouse), status.key);
						if (controller.getTarget() != null){
							status.target = controller.getTarget(); 
						}
						recorded.add(new RecordData(result.first, result.second, status));
						OgreJNI.setMotion(result.first);
						OgreJNI.setBall(Utils.singleList(result.second));
						frame++;
					}
				}
			}
		} catch (RuntimeException e){
			e.printStackTrace();
		}
		save(recorded);
		OgreJNI.close();
		System.exit(0);
	}
	
	private void save(ArrayList<RecordData> recorded){
		if (recorded.size() == 0) return;
		
		int persons = recorded.get(0).motion.length;
		String folder = OUTPUT_FOLDER + "\\" + controller.name;
		new File(folder).mkdirs();
		BVHWriter writer = new BVHWriter(new File(Configuration.BASE_MOTION_FILE));
		for (int i = 0; i < persons; i++) {
			ArrayList<Motion> motionList = new ArrayList<Motion>();
			for (RecordData r : recorded){
				motionList.add(r.motion[i]);
			}
			System.out.println("write motion :: " + motionList.size() + " : " + recorded.size());
			for (int j = 0; j < motionList.size(); j++) {
				motionList.get(j).knot = j;
			}
			MotionData mData = new MotionData(motionList);
			writer.write(new File(folder + "\\" + motionFile(i)), mData);
		}
		
		String[] ballTrajectory = new String[recorded.size()];
		String[] statusList = new String[recorded.size()];
		for (int i = 0; i < ballTrajectory.length; i++) {
			Point3d p = recorded.get(i).ball;
			if (p == null) p = new Point3d(-100000, -100000, -100000);
			ballTrajectory[i] = Utils.toString(p.x, p.y, p.z);
			statusList[i] = Utils.toString(recorded.get(i).status.toString());
		}
		FileUtil.writeAsString(ballTrajectory, folder + "\\" + BALL_FILE);
		FileUtil.writeAsString(statusList, folder + "\\" + STATUS_FILE);
	}
	
	public static String motionFile(int index){
		return "person_" + (index + 1) + ".bvh";
	}
	
	public static class RecordData{
		Motion[] motion;
		Point3d ball;
		OgreStatus status;
		
		public RecordData(Motion[] motion, Point3d ball, OgreStatus status) {
			this.motion = motion;
			this.ball = ball;
			this.status = status;
		}
	}
	
}
