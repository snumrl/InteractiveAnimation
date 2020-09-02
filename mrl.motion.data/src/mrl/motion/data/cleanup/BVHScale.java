package mrl.motion.data.cleanup;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.file.Files;

public class BVHScale {
	
	
	
	public static void scale(File file, double scale) throws IOException{
		File tempFile = new File(file.getAbsoluteFile().getParent() + "\\" + "_" + file.getName());
		BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(file)));
		BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(tempFile)));
		System.out.println("file : " + file.getName());
		String line;
		boolean isStarted = false;
		while ((line = br.readLine()) != null) {
			line = line.trim();
			if (line.trim().length() == 0) continue;
			if (line.trim().startsWith("OFFSET ")){
				String[] tokens = line.trim().split(" ");
				line = line.substring(0, line.indexOf("OFFSET")) + "OFFSET " + String.format("%.6f %.6f %.6f", 
						Double.parseDouble(tokens[1]) * scale,Double.parseDouble(tokens[2]) * scale,Double.parseDouble(tokens[3]) * scale);
			}
			if (isStarted && line.trim().length() > 0){
				int i1 = line.indexOf(" ");
				int i2 = line.indexOf(" ", i1+1);
				int i3 = line.indexOf(" ", i2+1);
				
				String[] tokens = line.split(" ");
				line = String.format("%.6f %.6f %.6f", 
						Double.parseDouble(tokens[0]) * scale,Double.parseDouble(tokens[1]) * scale,Double.parseDouble(tokens[2]) * scale) + line.substring(i3);
			}
			
			bw.write(line + "\r\n");
			
			if (line.startsWith("Frame Time:")) isStarted = true;
		}
		bw.close();
		br.close();
		
		file.delete();
		tempFile.renameTo(file);
	}

	public static void main(String[] args) {
//		{
//			Vector3d v = new Vector3d(24,-10,50);
//			Matrix3d m = eulerToMatrixZXY(v);
//			Vector3d e = matrixToEulerZXY(m);
//			System.out.println(v);
//			System.out.println(e);
//			System.exit(0);
//		}
//		
//		
		try{
			scale(new File("D:\\data\\Tennis\\test\\test.bvh"), 2.66);
//			scale(new File("D:\\data\\Tennis\\tennis_mocapdata.com\\bvh\\test.bvh"), 2.66);
			System.exit(0);
//			scale(new File("Trial001.bvh"), 100);
//			scale(new File("Trial002.bvh"), 100);
//			scale(new File("Trial004.bvh"), 100);
//			System.exit(0);
//			scale(new File("bvhOutput\\KMS.bvh"), 100);
//			scale(new File("bvhOutput\\SDH.bvh"), 100);
//			scale(new File("bvhOutput\\YDS.bvh"), 100);
//			System.exit(0);
			
			
//			File folder = new File("D:\\data\\AniCourse\\cmu_not_retarget - บนป็บป");
			File folder = new File("D:\\data\\motionsynth\\cmu_copy");
			for (File file : folder.listFiles()){
				if (file.isDirectory()) continue;
				if (file.getName().startsWith("_")) continue;
				if (!file.getName().endsWith("bvh")) continue;
//				scale(file, 100);
//				scale(file, 6);
				
				File rename = new File(file.getAbsolutePath().replace(".bvh", "") + "_1.bvh");
				file.renameTo(rename);
				scale(rename, 6);
			}
		} catch (IOException e){
			e.printStackTrace();
		}
	}
}
