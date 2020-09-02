package mrl.motion.neural.param;

import java.io.File;
import java.util.HashMap;
import java.util.HashSet;

import mrl.motion.data.MDatabase;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.MotionListModule;
import mrl.util.Utils;
import mrl.widget.app.MainApplication;

public class DuplicateMotionRemover {

	
	private MDatabase database;

	public DuplicateMotionRemover(MDatabase database) {
		this.database = database;
	}
	
	
	public void calc(){
		
	}
	
	private static void copy(){
		int[] indices = { 2,3,4,6,9,13,16,18,22,25,28,31,34,37,39,41,42,50,51,56,59,62,65 };
		HashSet<Integer> set = new HashSet<Integer>();
		for (int i : indices){
			set.add(i);
		}
		File folder = new File("walk");
		for (File f : folder.listFiles()){
			int index = Integer.parseInt(f.getName().substring(3, 5));
			if (!set.contains(index)){
				f.delete();
			}
		}
		
	}
	
	
	public static void main(String[] args) {
		copy();
		System.exit(0);
		MDatabase database = RNNDataGenerator.loadCMUDatabase("walk");
		System.out.println(database.getMotionList().length);
		for (int i = 0; i < database.getMotionDataList().length; i++) {
			System.out.println(i + "\t" + database.getMotionDataList()[i].motionList.size());
		}
		
		MainApplication app = new MainApplication();
		app.initializeModules();
		app.getModule(MainViewerModule.class);
		app.getModule(MotionListModule.class).setDatabase(database);
		app.open();
	}
}
