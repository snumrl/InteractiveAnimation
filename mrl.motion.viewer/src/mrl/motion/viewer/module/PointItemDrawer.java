package mrl.motion.viewer.module;

import javax.media.opengl.GL;
import javax.media.opengl.glu.GLU;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.viewer.SWTViewableCanvas;
import mrl.widget.app.Item;

import com.sun.opengl.util.GLUT;

public class PointItemDrawer extends ItemDrawer{

	@Override
	public boolean isDrawable(Object data) {
		return data instanceof Point3d;
	}

	@Override
	public void draw(GL gl, GLU glu, GLUT glut, Item item, Object data, int timeIndex) {
		Point3d p = (Point3d)data;
		Vector3d color = item.getColor(new Vector3d(0.2, 0.8, 0.2));
		gl.glColor3d(color.x, color.y, color.z);
		SWTViewableCanvas.drawSphere(gl, glut, p, item.getSize(5));
	}

}
