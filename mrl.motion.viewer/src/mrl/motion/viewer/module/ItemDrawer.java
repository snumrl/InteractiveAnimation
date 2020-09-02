package mrl.motion.viewer.module;

import javax.media.opengl.GL;
import javax.media.opengl.glu.GLU;

import com.sun.opengl.util.GLUT;

import mrl.widget.app.Item;

public abstract class ItemDrawer {

	public abstract boolean isDrawable(Object data);
	public abstract void draw(GL gl, GLU glu, GLUT glut, Item item, Object data, int timeIndex);
	
	public int getTimeLength(Object data){
		return -1;
	}
	
	public String toString(Item item, Object data, int timeIndex){
		return null;
	}
}
