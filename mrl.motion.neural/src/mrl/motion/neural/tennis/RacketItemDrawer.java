package mrl.motion.neural.tennis;

import javax.media.opengl.GL;
import javax.media.opengl.glu.GLU;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.sun.opengl.util.GLUT;

import mrl.motion.neural.tennis.TennisRacketData.TennisRacket;
import mrl.motion.viewer.SWTViewableCanvas;
import mrl.motion.viewer.module.ItemDrawer;
import mrl.widget.app.Item;

public class RacketItemDrawer extends ItemDrawer {

	@Override
	public boolean isDrawable(Object data) {
		return data instanceof TennisRacketData;
	}

	@Override
	public void draw(GL gl, GLU glu, GLUT glut, Item item, Object data, int timeIndex) {
		TennisRacketData rData = (TennisRacketData)data;
		if (timeIndex < 0 || timeIndex >= rData.racketList.size()) return;
		Vector3d color = item.getColor(new Vector3d(0, 0, 1));
		gl.glColor4d(color.x, color.y, color.z, 1);
		
		TennisRacket racket = rData.racketList.get(timeIndex);
		for (int i = 0; i < racket.points.length; i++) {
			Point3d p1 = racket.points[i];
			Point3d p2 = racket.points[(i+1)%racket.points.length];
			SWTViewableCanvas.drawLine(gl, glut, p1, p2, 3);
		}
	}

}
