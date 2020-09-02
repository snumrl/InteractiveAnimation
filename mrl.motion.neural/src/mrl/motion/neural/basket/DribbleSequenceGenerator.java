package mrl.motion.neural.basket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import mrl.motion.data.clip.MClip;
import mrl.motion.data.clip.MClipManager;
import mrl.util.MathUtil;
import mrl.util.PrintCounter;

public class DribbleSequenceGenerator {

	private ArrayList<MClip> clipList;
	
	private boolean[][] isConnectable;
	private int[][] linkGenerated;
	private int[] nodeGenerated;
	private int totalCount;
	private int generatedCount;
	
	private Random rand = MathUtil.random;
	
	public DribbleSequenceGenerator(MClipManager cManager){
		clipList = cManager.labelMap.get("dribble");
		int size = clipList.size();
		nodeGenerated = new int[size];
		linkGenerated = new int[size][size];
		System.out.println("clip size :: " + size);
		isConnectable = new boolean[size][size];		
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				double d = cManager.distanceMap[clipList.get(i).index][clipList.get(j).index].distance;
				if ((d < 10)){
					isConnectable[i][j] = true;
					totalCount++;
				}
			}
		}
	}
	
	public ArrayList<MClip> generate(int size){
		int frames = 0;
		ArrayList<MClip> sequence = new ArrayList<MClip>();
		
		PrintCounter pc = new PrintCounter(1000); 
		int current = rand.nextInt(clipList.size());
		while (true){
			sequence.add(clipList.get(current));
			nodeGenerated[current]++;
			frames += clipList.get(current).length();
			generatedCount++;
			
			pc.print(frames, "genenerated :: " + generatedCount + " / " + totalCount + " :: " + frames);
			if (frames > size) break;
			
			int next = getNextIndex(current);
			linkGenerated[current][next]++;
			current = next;
		}
		
		int zeroCount = 0;
		for (int c : nodeGenerated){
			if (c == 0) zeroCount++;
		}
		System.out.println("zero Count :: " + zeroCount + " / " + nodeGenerated.length + " :: " + zeroCount/(double)nodeGenerated.length);
		System.out.println(Arrays.toString(MathUtil.getStatistics(MathUtil.toDouble(nodeGenerated))));
		return sequence;
	}
	
	
	private int getNextIndex(int start){
		double pSum = 0;
		double[] accList = new double[nodeGenerated.length];
		for (int i = 0; i < nodeGenerated.length; i++) {
			double p;
			if (isConnectable[start][i]){
				int count = nodeGenerated[i] + linkGenerated[start][i];
				p = 1d/(count + 1);
			} else {
				p = 0;
			}
			p = p*p;
			pSum += p;
			accList[i] = pSum;
		}
		double p = rand.nextDouble()*pSum;
		for (int i = 0; i < accList.length; i++) {
			if (p <= accList[i]){
				return i;
			}
		}
		throw new RuntimeException();
	}
	
}
