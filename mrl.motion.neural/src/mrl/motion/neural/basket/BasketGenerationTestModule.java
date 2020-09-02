package mrl.motion.neural.basket;

import java.util.ArrayList;
import java.util.Random;

import mrl.motion.data.MDatabase;
import mrl.motion.data.MotionData;
import mrl.motion.data.clip.MClip;
import mrl.motion.data.clip.MClipManager;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.FileUtil;
import mrl.util.MathUtil;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;

public class BasketGenerationTestModule extends Module{

	public static String CACHE_FILE = "basket_sequence.dat";
	
	private MDatabase database;
	private MClipManager cManager;
	private ArrayList<MClip> clipList;
	
	@Override
	protected void initializeImpl() {
		database = BasketDataGenerator.loadBasketData();
		cManager = new MClipManager(database, BasketDataGenerator.CLIP_FILE);
		getModule(MainViewerModule.class);
		
		addMenu("&Basket Sequence", "&Generate\tCtrl+G", SWT.MOD1 + 'G', new Runnable() {
			@Override
			public void run() {
				BasketSequenceGenerator g = new BasketSequenceGenerator(cManager);
				clipList = g.generate(200);
				updateClipList();
			}
		});
		addMenu("&Basket Sequence", "&Load\tCtrl+L", SWT.MOD1 + 'L', new Runnable() {
			@Override
			public void run() {
				@SuppressWarnings("unchecked")
				ArrayList<Integer> list = (ArrayList<Integer>)FileUtil.readObject(CACHE_FILE);
				clipList = new ArrayList<MClip>();
				for (int index : list){
					clipList.add(cManager.totalClips.get(index));
				}
				updateClipList();
			}
		});
		addMenu("&Basket Sequence", "&Save\tCtrl+S", SWT.MOD1 + 'S', new Runnable() {
			@Override
			public void run() {
				ArrayList<Integer> list = new ArrayList<Integer>();
				for (MClip clip : clipList){
					list.add(clip.index);
				}
				FileUtil.writeObject(list, CACHE_FILE);
			}
		});
	}
	
	private void updateClipList(){
		MathUtil.random.setSeed(new Random().nextLong());
		MotionSegment segment = cManager.generateMotion(clipList);
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam(); 
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		getModule(ItemListModule.class).addSingleItem("basket", new MotionData(segment.getMotionList()));
	}

	public static void main(String[] args) {
		MainApplication.run(new BasketGenerationTestModule());
	}
}
