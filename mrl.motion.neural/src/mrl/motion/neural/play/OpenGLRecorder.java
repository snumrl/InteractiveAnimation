package mrl.motion.neural.play;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.MainViewerModule.MainViewer;
import mrl.util.Pair;
import mrl.util.Utils;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;
import mrl.widget.app.Item.ItemDescription;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;

public class OpenGLRecorder extends Module{
	
	
	private RuntimeController c;

	
	public OpenGLRecorder(RuntimeController c) {
		this.c = c;
	}

	@Override
	protected void initializeImpl() {
		getModule(MainViewerModule.class);
		c.app = app();
		c.init();
		
		getModule(ItemListModule.class).addSingleItem("goalPos", BallTrajectoryGenerator.GOAL_POS, new ItemDescription(new Vector3d(0,1,1)));
		
		addMenu("&Menu", "Test &Target\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				start();
			}
		});
	}
	
	private int currentKey = -1;
	private void start(){
		MainViewer viewer = getModule(MainViewerModule.class).getMainViewer();
		viewer.getCanvas().addKeyListener(new KeyListener() {
			@Override
			public void keyReleased(KeyEvent e) {
				currentKey = -1;
			}
			
			@Override
			public void keyPressed(KeyEvent e) {
				currentKey = (int)e.character;
			}
		});
		
		dummyParent().getDisplay().timerExec(1, new Runnable() {
			int frame = -1;
			long startTime = -1;
			@Override
			public void run() {
				if (dummyParent().isDisposed()) return;
				
				if (startTime < 0){
					startTime = System.currentTimeMillis();
					frame = 0;
				}
				
				while (true){
					int dt = (int)(System.currentTimeMillis() - startTime);
					int tIndex = dt/33;
					if (frame > tIndex) break;
					
					Point3d target3d = getModule(MainViewerModule.class).getPickPoint();
					if (target3d == null) break;
					Point2d target = Pose2d.to2d(target3d);
					Pair<Motion[], Point3d> result = c.process(frame, target, currentKey);
					if (c.omitTime > 0){
						startTime += c.omitTime;
						c.omitTime = 0;
					}
					
					Motion[] motions = result.first;
					for (int i = 0; i < motions.length; i++) {
						getModule(ItemListModule.class).addSingleItem("Motion" + i, new MotionData(Utils.singleList(motions[i])));
					}
					getModule(ItemListModule.class).addSingleItem("Ball", result.second, BallTrajectoryGenerator.ballDescription());
					frame++;
				}
				dummyParent().getDisplay().timerExec(1, this);
			}
		});
	}
	
	
	
	public static void run(RuntimeController c){
		MainApplication.run(new OpenGLRecorder(c));
	}
}
