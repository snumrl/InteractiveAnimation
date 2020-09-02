package mrl.motion.data.clip;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.graph.MotionSegment;

public class MClipManager {

	public MDatabase database;
	public ArrayList<MClip> totalClips;
	public HashMap<String, ArrayList<MClip>> labelMap;
	public MClipPair[][] distanceMap;
	
	public MClipManager(MDatabase database, String cacheFile){
		this.database = database;
		totalClips = MClip.getClipList(database, database.getEventAnnotations());
		labelMap = new HashMap<String, ArrayList<MClip>>();
		for (MClip clip : totalClips) {
			ArrayList<MClip> list = labelMap.get(clip.type);
			if (list == null) {
				list = new ArrayList<MClip>();
				labelMap.put(clip.type, list);
			}
			list.add(clip);
		}
		distanceMap = MClipPair.load(new File(cacheFile));
	}

	public MotionSegment generateMotion(ArrayList<MClip> clipList){
		ArrayList<MotionSegment> segmentList = new ArrayList<MotionSegment>();
		Motion[] mList = database.getMotionList();
		int startOffset = 0;
		ArrayList<Integer> offsetList = new ArrayList<Integer>();
		for (int i = 0; i < clipList.size(); i++) {
			MClip clip = clipList.get(i);
			MClipPair pair = null;
			if (i < clipList.size() - 1){
				pair = distanceMap[clip.index][clipList.get(i+1).index];
			}
			int endOffset = (pair == null) ? 0 : pair.sourceOffset;
			
			offsetList.add(startOffset);
			segmentList.add(new MotionSegment(mList, clip.startMotionIndex + startOffset, clip.lastMotionIndex + endOffset));
			if (pair != null){
				startOffset = pair.targetOffset;
			}
		}
		
		MotionSegment merged = null;
		for (MotionSegment s : segmentList){
			if (merged == null){
				merged = s;
			} else {
				merged = MotionSegment.stitch(merged, s, true);
			}
		}
		return merged;
	}
	
	public MClip findClip(String file, int startFrame){
		file = file.replace(".bvh", "");
		for (MClip clip : totalClips){
			if (clip.file.equals(file) && clip.startFrame == startFrame) return clip;
		}
		return null;
	}
}
