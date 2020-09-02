package mrl.motion.viewer.module;

import java.util.HashMap;

import javax.vecmath.Point3d;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.Motion;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.trasf.Pose2d;
import mrl.widget.app.Item;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;

import org.eclipse.swt.SWT;

public class MotionTestApp extends MainApplication{

	public MotionTestApp(){
		addModule(new MainViewerModule());
	}
	
	protected void init(){
//		FootContactDetection.heightLimit = new double[]{
//			25, 15, 15
//		};
//		FootContactDetection.velocityLimit = new double[]{
//				3, 3, 3
//		};
		FootContactDetection.rightFootJoints = new String[]{ "RightFoot", "RightToeBase", "RightToeBase_End" } ;
		FootContactDetection.leftFootJoints = new String[]{ "LeftFoot", "LeftToeBase", "LeftToeBase_End" } ;
		MotionListModule listModule = getModule(MotionListModule.class);
//		listModule.loadMotionFolder("C:\\Users\\khlee\\git\\MotionGAN\\mrl.motion.neural\\salsa\\motion");
		
//		listModule.loadMotionFolder("D:\\Dev\\MotionGAN_seyoung\\MotionGAN\\mrl.motion.neural\\tennisData\\motion");
		
		listModule.loadMotionFolder("C:\\Users\\khlee\\git\\MotionGAN\\mrl.motion.neural\\locomotion\\edin_loco\\motion");
		
//		listModule.loadMotionFolder("C:\\Users\\khlee\\git\\MotionGAN\\mrl.motion.neural\\basketData\\motion");
		
		Pose2d p = Pose2d.BASE;
		Item item = new Item(p);
		item.setLabel("Pose");
		getModule(ItemListModule.class).addItem(item);
//		listModule.loadMotionFolder("C:\\Dev\\workspace-new\\mrl.test.neural\\motionFolder");
		
		addMenu("&Menu", "Test &Target\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				int tIndex = getModule(MainViewerModule.class).getTimeIndex();
				Motion motion = getModule(MotionListModule.class).getSelectedMotionData()[0].motionList.get(tIndex);
				HashMap<String, Point3d> points = Motion.getPointData(SkeletonData.instance, motion);
				for (String j : FootContactDetection.leftFootJoints){
					System.out.println(j + " : " + points.get(j));
				}
				for (String j : FootContactDetection.rightFootJoints){
					System.out.println(j + " : " + points.get(j));
				}
			}
		});
	}
	
	public static void main(String[] args) {
		new MotionTestApp().open();
	}
}
