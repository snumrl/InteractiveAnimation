package mrl.motion.viewer.module;

import java.util.HashMap;

import javax.media.opengl.GL;
import javax.media.opengl.glu.GLU;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.Motion;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.position.PositionResultMotion.PositionFrame;
import mrl.motion.viewer.SWTViewableCanvas;
import mrl.widget.app.Item;

import com.sun.opengl.util.GLUT;

public class PositionMotionItemDrawer extends ItemDrawer{

	@Override
	public boolean isDrawable(Object data) {
		return (data instanceof PositionResultMotion);
	}

	@Override
	public void draw(GL gl, GLU glu, GLUT glut, Item item, Object data, int timeIndex) {
		PositionResultMotion motion = (PositionResultMotion)data;
		if (timeIndex < 0 || timeIndex >= motion.size()) return;
		
		PositionFrame positionList = motion.get(timeIndex);
		double boneThickness = MotionItemDrawer.BoneThickness;
		Vector3d c = item.getColor(new Vector3d(0.8, 0.2, 0.2));
		gl.glColor3d(c.x, c.y, c.z);
		for (Point3d[] pair : positionList){
			SWTViewableCanvas.drawLine(gl, glut, pair[0], pair[1], boneThickness);
			gl.glPushMatrix();
			gl.glTranslated(pair[0].x, pair[0].y, pair[0].z);
			glut.glutSolidSphere((boneThickness + 0.05), 20, 20);
			gl.glPopMatrix();
			gl.glPushMatrix();
			gl.glTranslated(pair[1].x, pair[1].y, pair[1].z);
			glut.glutSolidSphere((boneThickness + 0.05), 20, 20);
			gl.glPopMatrix();
		}
		
		if (MotionItemDrawer.drawFootContact){
//			3, 11
			gl.glColor3d(0.2, 0.8, 0.2);
			if (positionList.footContact.left){
				Point3d[] pair = positionList.get(3);
				SWTViewableCanvas.drawSphere(gl, glut, pair[0], boneThickness + 2);
				SWTViewableCanvas.drawSphere(gl, glut, pair[1], boneThickness + 2);
			}
			if (positionList.footContact.right){
				Point3d[] pair = positionList.get(11);
				SWTViewableCanvas.drawSphere(gl, glut, pair[0], boneThickness + 2);
				SWTViewableCanvas.drawSphere(gl, glut, pair[1], boneThickness + 2);
			}
		}
	}

	public int getTimeLength(Object data){
		return ((PositionResultMotion)data).size();
	}
}
