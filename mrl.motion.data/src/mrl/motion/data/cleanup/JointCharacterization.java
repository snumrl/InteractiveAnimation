package mrl.motion.data.cleanup;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class JointCharacterization {

	public static String[][] aniCourse = {
			{"pelvis", "Hips"},
			{"lfemur", "LeftUpLeg"},
			{"ltibia", "LeftLeg"},
			{"lfoot", "LeftFoot"},
			{"rfemur", "RightUpLeg"},
			{"rtibia", "RightLeg"},
			{"rfoot", "RightFoot"},
			{"thorax", "Spine"},
			{"lhumerus", "LeftArm"},
			{"lradius", "LeftForeArm"},
			{"lhand", "LeftHand"},
			{"rhumerus", "RightArm"},
			{"rradius", "RightForeArm"},
			{"rhand", "RightHand"},
	};
	
	public static void apply(String folder, String[][] mapping){
		try {
			for (File file : new File(folder).listFiles()){
				if (!file.getName().toLowerCase().endsWith(".bvh")) continue;
				
				File tempFile = new File(file.getAbsoluteFile().getParent() + "\\" + "_" + file.getName());
				
				BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(file)));
				BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(tempFile)));
				System.out.println("file : " + file.getName());
				String line;
				while ((line = br.readLine()) != null) {
					for (String[] map : mapping){
						line = line.replace(map[0], map[1]);
					}
					bw.write(line + "\r\n");
				}
				bw.close();
				br.close();
				
				file.delete();
				tempFile.renameTo(file);
			}
			
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
	}
	
	
	public static void main(String[] args) {
		apply("D:\\data\\AniCourse\\output", aniCourse);
	}
	
}
