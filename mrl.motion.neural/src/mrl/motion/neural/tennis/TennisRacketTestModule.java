package mrl.motion.neural.tennis;

import mrl.motion.data.MDatabase;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

public class TennisRacketTestModule extends Module{

	@Override
	protected void initializeImpl() {
		MainViewerModule mainViewer = getModule(MainViewerModule.class);
		mainViewer.addDrawer(new RacketItemDrawer());
		
		MDatabase database = TennisDataGenerator.loadTennisData("tennisData\\motion");
		MotionData mData = database.getMotionDataList()[0];
		ItemListModule itemList = getModule(ItemListModule.class);
		itemList.addSingleItem("motion", mData);
		itemList.addSingleItem("racket", new TennisRacketData(mData));
	}

	
	public static void main(String[] args) {
		MainApplication.run(new TennisRacketTestModule());
	}
}
