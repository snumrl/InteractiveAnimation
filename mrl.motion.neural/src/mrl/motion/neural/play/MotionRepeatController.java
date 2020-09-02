package mrl.motion.neural.play;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.cmu.CMUClip;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.util.Pair;

public class MotionRepeatController extends RuntimeController{
	
	private MotionData motionData;

	public MotionRepeatController(String name) {
		super(name);
	}

	@Override
	public void init() {
		MotionSegment.alignToBase(motionData.motionList);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		System.out.println("init!!");
	}

	@Override
	public Pair<Motion[], Point3d> process(int frame, Point2d mousePoint, int key) {
		int div = motionData.motionList.size() + 6;
		int index;
		index = Math.min(motionData.motionList.size()-1, frame % div);
		return new Pair<Motion[], Point3d>(new Motion[]{new Motion(motionData.motionList.get(index)) }, null);
	}
	
	
	public static void main(String[] args) {
		MotionDataConverter.setNoBall();
		
		MotionRepeatController c = new MotionRepeatController("cmu_clip_1");
		CMUClip clip = new CMUClip();
//		c.motionData = new MotionData(clip.segments.get(0).getMotionList());
		c.motionData = new MotionData(clip.walk.getMotionList());
		new OgreRecorder(c).run(new double[]{ 0 });
	}

}
