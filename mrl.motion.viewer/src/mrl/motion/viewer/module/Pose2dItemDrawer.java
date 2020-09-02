package mrl.motion.viewer.module;

import javax.media.opengl.GL;
import javax.media.opengl.glu.GLU;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.viewer.SWTViewableCanvas;
import mrl.widget.app.Item;

import com.sun.opengl.util.GLUT;

public class Pose2dItemDrawer extends ItemDrawer{

	@Override
	public boolean isDrawable(Object data) {
		return data instanceof Pose2d;
	}

	@Override
	public void draw(GL gl, GLU glu, GLUT glut, Item item, Object data, int timeIndex) {
		Pose2d p = (Pose2d)data;
		Vector3d color = item.getColor(new Vector3d(0.2, 0.2, 0.8));
		gl.glColor3d(color.x, color.y, color.z);
		
		double s = item.getSize(5)/5;
		
		Point3d p1 = p.position3d();
		Vector3d direction = p.direction3d();
		direction.scale(15*s*2);
		Point3d p2 = new Point3d();
		p2.add(p1, direction);
		SWTViewableCanvas.drawSphere(gl, glut, p1, 5*s);
		SWTViewableCanvas.drawLine(gl, glut, p1, p2, 3*s);
	}

}
