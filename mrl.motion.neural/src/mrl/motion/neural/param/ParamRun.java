package mrl.motion.neural.param;

import java.util.ArrayList;

import mrl.motion.data.MDatabase;
import mrl.motion.data.MotionData;
import mrl.motion.data.clip.MClip;
import mrl.motion.data.clip.MClipManager;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.basket.BasketGraph;
import mrl.motion.neural.basket.BasketSequenceGenerator;
import mrl.motion.neural.basket.DribbleSequenceGenerator;
import mrl.motion.neural.basket.BasketGraph.BasketNode;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;

public class ParamRun {
	private MainApplication app;
	
	public ParamRun(){
		app = new MainApplication();
		
//		app.addModule(new TransformParamModule());
		app.addModule(new MotionEditModule());
//		app.addModule(new TransformViewerModule());
//		app.addModule(new ParamTestModule());
		
		
//		{
//			MDatabase database = BasketDataGenerator.loadBasketData();
//			MClipManager cManager = new MClipManager(database, BasketDataGenerator.CLIP_FILE);
//			
////			DribbleSequenceGenerator g = new DribbleSequenceGenerator(cManager);
////			ArrayList<MClip> clipList = g.generate(2000);
//			
//			BasketSequenceGenerator g = new BasketSequenceGenerator(cManager);
//			ArrayList<MClip> clipList = g.generate(200);
//			
//			MotionSegment segment = cManager.generateMotion(clipList);
//			TrajectoryEditParam tEdit = new TrajectoryEditParam(); 
//			segment = tEdit.edit(segment);
//			FootSlipCleanup.clean(segment);
//			app.addModule(new MainViewerModule());
//			app.getModule(ItemListModule.class).addSingleItem("basket", new MotionData(segment.getMotionList()));
//		}
		
		app.open();
	}
	

	public static void main(String[] args) {
		new ParamRun();
	}
}
