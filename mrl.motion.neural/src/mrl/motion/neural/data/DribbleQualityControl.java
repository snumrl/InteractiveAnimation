package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point3d;

import mrl.motion.data.Contact;
import mrl.motion.data.Motion;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.position.PositionMotion;

public class DribbleQualityControl extends ControlDataGenerator {

	private ArrayList<HashMap<String, Point3d>> jointPositions = new ArrayList<HashMap<String,Point3d>>();
	private String type;
	
	public DribbleQualityControl(String type){
		this.type = type;
		includeRootMove = true;
	}
	
	protected void initImpl(){
		for (int i = 0; i < mList.size(); i++) {
			Motion m = mList.get(i);
			PositionMotion pm = new PositionMotion(m);
			jointPositions.add(pm.pointData);
		}
	}
	
	@Override
	public double[] getControl(int index) {
		int interval = 30;
		
		int controlIndex = index + interval;
		if (controlIndex >= mList.size() - interval) return null;
		
		Pose2d p = PositionMotion.getPose(mList.get(index));
		Pose2d cp = PositionMotion.getPose(mList.get(controlIndex));
		Pose2d targetPose = Pose2d.relativePose(p, cp);
		
//		"rm", "rh", "ho", "ha", "hc"
		// root move
		// hand move
		// root height
		
		// ball cycle interval
		// ball have time
		
		// hand change
		double control;
		if (type.equals("rm")){
			control = getMovement(rootMove, index);
		} else if (type.equals("rh")){
			control = rootHeight(index);
		} else if (type.equals("ho")){
			control = handOutside(index);
		} else if (type.equals("ha")){
			control = handAlternate(index);
		} else if (type.equals("hc")){
			control = handCycle(index);
		} else {
			throw new RuntimeException();
		}
		
		return new double[]{ targetPose.position.x, targetPose.position.y, control };
	}
	
	private double handOutside(int index){
		int margin = 30;
		int count = 0;
		double sum = 0;
		for (int i = -margin; i <= margin; i++) {
			int idx = index + i;
			if (i < 0 || i >= mList.size()) continue;
			
			for (String j : BallTrajectoryGenerator.HAND_JOINTS){
				Point3d p = jointPositions.get(idx).get(j);
				sum += Math.abs(p.z);
				count++;
			}
		}
		return sum/count;
	}
	
	private double rootHeight(int index){
		int margin = 30;
		int count = 0;
		double sum = 0;
		for (int i = -margin; i <= margin; i++) {
			int idx = index + i;
			if (i < 0 || i >= mList.size()) continue;
			
			Point3d p = jointPositions.get(idx).get(SkeletonData.instance.root.name);
			sum += Math.abs(p.y);
			count++;
		}
		return sum/count;
	}
	
	private double handAlternate(int index){
		Boolean[] contacts = new Boolean[3];
		Contact base = mList.get(index).ballContact;
		
		
		int leftStart = index;
		int rightStart = index;
		if (!base.isNoContact()){
			contacts[1] = base.left;
			while (true){
				if (leftStart <= 0) break;
				leftStart--;
				if (mList.get(leftStart).ballContact.isNoContact()) break;
			}
		} else {
			while (true){
				if (rightStart >= mList.size() - 1) break;
				rightStart++;
				if (!mList.get(rightStart).ballContact.isNoContact()) break;
			}
			base =  mList.get(rightStart).ballContact;
			contacts[1] = base.left;
		}
		
		if (base.left && base.right) return 0;
		
		int i = leftStart;
		Boolean isLeft = null;
		while (true){
			if (i <= 0) break;
			i--;
			Contact c = mList.get(i).ballContact;
			if (c.isNoContact()) continue;
			if (c.left && c.right) break;
			isLeft = c.left;
			break;
		}
		contacts[0] = isLeft;
		
		while (true){
			if (rightStart >= mList.size() - 1) break;
			rightStart++;
			if (mList.get(rightStart).ballContact.isNoContact()) break;
		}
		while (true){
			if (rightStart >= mList.size() - 1) break;
			rightStart++;
			if (!mList.get(rightStart).ballContact.isNoContact()) break;
		}
		contacts[2] =  isLeft(mList.get(rightStart).ballContact);
		
		double change = 0;
		for (int j = 0; j < contacts.length - 1; j++) {
			if (contacts[j] == null || contacts[j+1] == null) continue;
			if (contacts[j] != contacts[j+1]) change++;
		}
		return change;
	}
	
	private Boolean isLeft(Contact c){
		if (c.left && c.right) return null;
		return c.left;
	}
	
	private double handCycle(int index){
		int left = index;
		{
			int changedCount = 0;
			boolean current = mList.get(index).ballContact.isNoContact();
			while (true){
				if (left <= 0) break;
				left--;
				boolean c = mList.get(left).ballContact.isNoContact();
				if (c != current){
					changedCount++;
					if (changedCount == 2) break;
				}
				current = c;
			}
		}
		int right = index;
		{
			int changedCount = 0;
			boolean current = mList.get(index).ballContact.isNoContact();
			while (true){
				if (right >= mList.size() - 1) break;
				right++;
				boolean c = mList.get(right).ballContact.isNoContact();
				if (c != current){
					changedCount++;
					if (changedCount == 3) break;
				}
				current = c;
			}
		}
		return (right - left)/2d;
	}
}
