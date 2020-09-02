package mrl.motion.neural.basket;

import java.io.File;
import java.util.ArrayList;

import javax.media.opengl.GL;
import javax.media.opengl.glu.GLU;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.annotation.MotionAnnotationHelper;
import mrl.motion.data.MotionAnnotation;
import mrl.motion.data.MotionData;
import mrl.motion.viewer.GLDrawListener;
import mrl.motion.viewer.SWTViewableCanvas;
import mrl.util.ObjectSerializer;

import org.eclipse.swt.SWT;

import com.sun.opengl.util.GLUT;

public class BasketAnnotationHelper extends MotionAnnotationHelper{
	
	private ArrayList<Point3d> ballTrajectory;
	private String subAnnFolder;

	public BasketAnnotationHelper(String dataFolder, String outputFolder, String subAnnFolder) {
		super(dataFolder, outputFolder);
		this.subAnnFolder = subAnnFolder;
	}

	protected void createInitialUI(){
		super.createInitialUI();
		
		motionViewer.getNavigator().getViewer().addDrawListener(new GLDrawListener() {
			@Override
			public void onDraw(GL gl, GLU glu, GLUT glut) {
				if (ballTrajectory == null) return;
				int index = motionViewer.getNavigator().getMotionIndex();
				if (ballTrajectory.size() <= index) return;
				Point3d p = ballTrajectory.get(index);
				Vector3d color = new Vector3d(0.2, 0.8, 0.2);
				gl.glColor3d(color.x, color.y, color.z);
				SWTViewableCanvas.drawSphere(gl, glut, p, BallTrajectoryGenerator.BALL_RADIUS);
			}
		});
	}
	
	protected void updateFileSelection(){
		super.updateFileSelection();
		
		File subAnnotationFile = new File(subAnnFolder + "\\" + currentAnnotationFile.getName());
		ArrayList<MotionAnnotation> subAnnotationList;
		if (subAnnotationFile.exists()){
			subAnnotationList = ObjectSerializer.load(MotionAnnotation.class, subAnnotationFile);
		} else {
			subAnnotationList = new ArrayList<MotionAnnotation>();
		}
		timeline.setSubAnnotationList(subAnnotationList);
	}
	
	@Override
	protected void createMenu() {
		super.createMenu();
		
		addMenu(submenu, "Update Ball Trajectory\tCtrl+B", SWT.MOD1 + 'B', new Runnable() {
			public void run() {
				saveCurrentAnnotation();
				int person = timeline.getSelectedPerson() - 1;
				MotionData[] mDataList = motionViewer.getNavigator().getViewer().getMotionDataList();
				MotionData mData = mDataList[person];
				String file = currentFileList.get(person).getName();
				BallTrajectoryGenerator.updateBallContacts(mData, file);
				ballTrajectory = new BallTrajectoryGenerator().generate(mData.motionList);
			}
		});
	}
}
