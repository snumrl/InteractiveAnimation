package mrl.motion.data.clip;

import java.io.File;
import java.util.ArrayList;

import mrl.motion.data.Contact;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.MotionVector;
import mrl.util.Configuration;
import mrl.util.IterativeRunnable;
import mrl.util.Utils;

public class MClipDistance {
	
	public static final int MAX_DISTANCE = 10000;
	private static int BALL_MIN_MARGIN = 6;
	
	private MotionTransform transform;
	private MClipPair[][] distanceMap;
	
	private Motion[] mList;
	private ClipNode[] nodeList;
	private int[] prevBallMargin;
	private int[] postBallMargin;
	private int[] ballLeftOffset;
	
	
	public MClipDistance(MDatabase database) {
		transform = new MotionTransform();
		mList = database.getMotionList();
		
		prevBallMargin = new int[mList.length];
		postBallMargin = new int[mList.length];
		ballLeftOffset = new int[mList.length];
		for (int i = 0; i < prevBallMargin.length; i++) {
			Contact contact = mList[i].ballContact;
			int margin = 0;
			for (int m = 1; m <= BALL_MIN_MARGIN; m++) {
				int idx = i - m;
				if (idx < 0) break;
				if (contact.equals(mList[idx].ballContact)){
					margin++;
				}
			}
			prevBallMargin[i] = margin;
			
			margin = 0;
			for (int m = 1; m <= BALL_MIN_MARGIN; m++) {
				int idx = i + m;
				if (idx >= mList.length) break;
				if (contact.equals(mList[idx].ballContact)){
					margin++;
				}
			}
			postBallMargin[i] = margin;
			
			ballLeftOffset[i] = ballLeftOffset(i, 10);
		}
		
		nodeList = new ClipNode[mList.length];
		for (int i = 0; i < mList.length; i++) {
			if (mList[i].next != null){
				nodeList[i] = new ClipNode(transform, i, mList[i]);
			}
		}
		
		
		final ArrayList<MClip> clipList = MClip.getClipList(database, database.getEventAnnotations());
		final int n = clipList.size();
		distanceMap = new MClipPair[n][n];
		final int[] progress = new int[1];
		Utils.runMultiThread(new IterativeRunnable() {
			@Override
			public void run(int i) {
				for (int j = 0; j < n; j++) {
					distanceMap[i][j] = getClipPair(clipList.get(i), clipList.get(j));
//					System.out.print(String.format("%.2f ", distanceMap[i][j].distance));
				}
//				System.out.println();
				int p;
				synchronized (progress) {
					progress[0]++;
					p = progress[0];
				}
				if ((p % 10) == 0){
					System.out.println("progress : " + p + " / " + n);
				}
			}
		}, n);
	}
	
	private int ballLeftOffset(int index, int margin){
		if (mList[index].ballContact.isNoContact()) return margin;
		for (int i = 1; i < margin; i++) {
			int idx = index + i;
			if (idx < 0 || idx >= mList.length) continue;
			if (mList[idx].ballContact.isNoContact()) return i;
		}
		return margin;
	}
	
	private MClipPair getClipPair(MClip sClip, MClip tClip){
		int sIdx = sClip.lastMotionIndex;
		int tIdx = tClip.startMotionIndex;
		MClipPair pair = new MClipPair();
		pair.distance = Integer.MAX_VALUE;
		
		int sMargin = Math.max(2, Math.min(MClip.MARGIN-1, (sClip.length()/4)));
		int tMargin = Math.max(2, Math.min(MClip.MARGIN-1, (tClip.length()/4)));
		
		// interaction frame 기준으로 앞뒤로 7 frame은 보장해주도록 함
		int iMargin = 11;
		int sOffset = 1;
		int tOffset = 1;
		if (sClip.isActionClip){
			sOffset = Math.min(sMargin, Math.max(0, sClip.length() - sClip.interactionOffset - iMargin));
		}
		if (tClip.isActionClip){
			tOffset = Math.min(tMargin, Math.max(0, tClip.interactionOffset - iMargin));
		}
		
		for (int i = -sOffset; i <= sMargin; i++) {
			ClipNode source = nodeList[sIdx + i];
			if (sIdx + i <= sClip.startMotionIndex + sClip.interactionOffset + 1) continue;
			for (int j = -tMargin; j <= tOffset; j++) {
				if (j >= tClip.interactionOffset - 1) continue;
				ClipNode target = nodeList[tIdx+j];
				double d;
				if (source.isAir || target.isAir){
					d = MAX_DISTANCE;
				} else {
					d = getDiff(nodeList[sIdx+i], nodeList[tIdx+j], sClip.type, tClip.type);
					d = Math.min(d, MAX_DISTANCE);
				}
				if (d < pair.distance){
					pair.distance = d;
					pair.sourceOffset = i;
					pair.targetOffset = j;
				}
			}
		}
		if (pair.distance == Integer.MAX_VALUE) throw new RuntimeException();
		if (sClip.isActionClip || tClip.isActionClip){
			pair.distance /= 3;
		}
		if (sClip.type.equals("dribble") && tClip.type.equals("pass")){
			pair.distance /= 2;
		}
		return pair;
	}
	
	public double getDiff(ClipNode source, ClipNode target, String sType, String tType){
		if (!MotionVector.isFootContactEqual(source.pose, target.pose)) return Integer.MAX_VALUE;
		
		int i = source.motion.motionIndex;
		int j = target.motion.motionIndex;
		
		if (sType.equals("dribble") && tType.equals("pass")){
			if (source.motion.ballContact.isNoContact() || target.motion.ballContact.isNoContact()) return Integer.MAX_VALUE;
		} else {
			if (!source.motion.ballContact.equals(target.motion.ballContact)) return Integer.MAX_VALUE;
			if (ballLeftOffset[i] <= 4 && ballLeftOffset[j] >= 8) return Integer.MAX_VALUE;
			if (ballLeftOffset[j] <= 4 && ballLeftOffset[i] >= 8) return Integer.MAX_VALUE;
			if (prevBallMargin[i+1] + postBallMargin[j] < BALL_MIN_MARGIN) return Integer.MAX_VALUE;
		}
		
		double dSum = 0;
		if (source.motion.ballContact.isNoContact()){
			dSum += Configuration.MGRAPH_EDGE_WEIGHT_LIMIT/4;
		}
		dSum += MotionVector.getMotionDistance(source.pose, target.pose, transform, dSum, Configuration.MGRAPH_EDGE_WEIGHT_LIMIT);
		dSum += MotionVector.getVectorDistance(source.velocity, target.velocity, transform, dSum, Configuration.MGRAPH_EDGE_WEIGHT_LIMIT);
		return dSum;
	}
	
	private static class ClipNode{
		public int index;
		public Motion motion;
		
		public MotionVector pose;
		public MotionVector velocity;
		public boolean isAir;
		
		public ClipNode(MotionTransform transform, int index, Motion motion) {
			this.index = index;
			this.motion = motion;
			
			pose = transform.toVector(motion);
			isAir = !motion.isLeftFootContact && !motion.isRightFootContact;
			velocity = MotionVector.getMotionVelocity(motion, motion.next, pose, transform.toVector(motion.next));
		}
		
	}
	
	public static void calcClipDistance(MDatabase database, String cacheFile){
		MClipDistance d = new MClipDistance(database);
		MClipPair.save(d.distanceMap, new File(cacheFile));
	}
	
//	public static void main(String[] args) {
//		Configuration.MAX_THREAD = 1;
//		MDatabase database = MDatabase.basket();
//		MClipDistance d = new MClipDistance(database);
//		MClipPair.save(d.distanceMap, new File(MClipPair.CACHE_FILE));
//	}
}
