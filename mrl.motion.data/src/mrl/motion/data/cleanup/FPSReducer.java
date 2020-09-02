package mrl.motion.data.cleanup;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class FPSReducer {
	
	public static void fpsReduce(File file){
		try {
			File tempFile = new File(file.getAbsoluteFile().getParent() + "\\" + "_" + file.getName());
			BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(file)));
			BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(tempFile)));
			System.out.println("file : " + file.getName());
			String line;
			boolean isStarted = false;
			int idx = 0;
			while ((line = br.readLine()) != null) {
				if (line.trim().length() == 0) continue;
				if (line.startsWith("Frames:")){
					int frames = Integer.parseInt(line.substring("Frames:".length() + 1));
					frames = (int)Math.ceil(frames/4d);
					line = "Frames:	" + frames;
				}
				
				if (isStarted){
					if ((idx % 4) == 0){
						bw.write(line + "\r\n");
					}
					idx++;
				} else {
					if (line.startsWith("Frame Time:")){
						isStarted = true;
						double frameTime = Double.parseDouble(line.substring("Frame Time:".length() + 1));
						frameTime = frameTime * 4;
						line = "Frame Time:	" + "0.0333333";
//						line = "Frame Time:	" + frameTime;
					}
					bw.write(line + "\r\n");
				}
			}
			bw.close();
			br.close();
			
			file.delete();
			tempFile.renameTo(file);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
	}
	
	public static void main(String[] args) {
		try{
			File folder = new File("C:\\Dev\\workspace-new\\mrl.test.motion.graph\\motionFolder2");
			
			for (File file : folder.listFiles()){
				if (file.isDirectory()) continue;
				if (file.getName().startsWith("_")) continue;
				if (!file.getName().endsWith("bvh")) continue;
				fpsReduce(file);
			}
			
		} catch (Exception e){
			e.printStackTrace();
		}
	}
}
