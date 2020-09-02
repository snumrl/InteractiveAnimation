package mrl.motion.data.clip;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

public class MClipPair {
	
	/** distance between two clips (10000 if they can't be connected) */
	public double distance;
	
	/** tail offset of previous motion */
	public int sourceOffset;
	/** head offset of next motion **/
	public int targetOffset;
	
	public int sourceIndex;
	public int targetIndex;
	
	MClipPair(){
	}
		
	public static void save(MClipPair[][] distanceMap, File file){
		try {
			ObjectOutputStream os = new ObjectOutputStream(new BufferedOutputStream(new FileOutputStream(file)));
			int version = 1;
			os.writeInt(version);
			
			int n = distanceMap.length;
			os.writeInt(n);
			
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					MClipPair pair = distanceMap[i][j];
					os.writeDouble(pair.distance);
					os.writeInt(pair.sourceOffset);
					os.writeInt(pair.targetOffset);
				}
			}
			
			os.close();
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
	}
	
	public static MClipPair[][] load(File file){
		try {
			ObjectInputStream oi = new ObjectInputStream(new BufferedInputStream(new FileInputStream(file)));
			/*int version = */oi.readInt();
			
			int n = oi.readInt();
			MClipPair[][] map = new MClipPair[n][n];
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					MClipPair pair = new MClipPair();
					pair.distance = oi.readDouble();
					pair.sourceOffset = oi.readInt();
					pair.targetOffset = oi.readInt();
					
					pair.sourceIndex = i;
					pair.targetIndex = j;
					map[i][j] = pair;
				}
			}
			
			oi.close();
			return map;
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
	}
	
}
