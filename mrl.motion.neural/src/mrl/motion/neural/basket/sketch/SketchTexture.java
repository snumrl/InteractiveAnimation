package mrl.motion.neural.basket.sketch;

import java.nio.ByteBuffer;

import javax.media.opengl.GL;

import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.ImageData;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.graphics.Transform;

public class SketchTexture {
	
	public static final int IMAGE_WIDTH = 1162;
	
	// court width 15m, height 14m
	public static final double COURT_SCALE = 1500d/(double)IMAGE_WIDTH; 
	
	
	public static void registerSketchTexture(SketchCanvas canvas, GL gl, double scale){
		Point courtSize = canvas.getCourtSize();
		Image image = new Image(canvas.getDisplay(), courtSize.x, courtSize.y);
		GC gc = new GC(image);
		Transform transform = new Transform(gc.getDevice());
		transform.translate(image.getBounds().width/2, 0);
		gc.setTransform(transform);
		canvas.drawContents(gc);
		ImageData imageData = image.getImageData();
		transform.dispose();
		gc.dispose();
		image.dispose();
		
		int w = courtSize.x;
		int h = courtSize.y;
		int n = w*h;
		byte[] RGBA = new byte[n*4];
		for (int i = 0; i < n; i++) {
			int idx = i*4;
			int value = imageData.getPixel(i%w, i/w);
			RGBA[idx + 2] =  (byte)((value >> 24) & 255);
			RGBA[idx + 1] =  (byte)((value >> 16) & 255);
			RGBA[idx + 0] =  (byte)((value >> 8) & 255);
			RGBA[idx + 3] =  (byte)((value) & 255);
		}
		
		ByteBuffer bb = ByteBuffer.wrap(RGBA);
		bb.position(0);
		bb.mark();
		gl.glBindTexture(GL.GL_TEXTURE_2D, 13);
		gl.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1);
		gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP);
		gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP);
		gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER,
				GL.GL_LINEAR);
		gl.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER,
				GL.GL_LINEAR);
		gl.glTexEnvf(GL.GL_TEXTURE_ENV, GL.GL_TEXTURE_ENV_MODE, GL.GL_REPLACE);
		gl.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA, w, h, 0, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, bb);
	}
	
	public static void drawSketchTexture(SketchCanvas canvas, GL gl, double scale){
		Point courtSize = canvas.getCourtSize();
		int w = courtSize.x;
		int h = courtSize.y;
		
		int left = 0;
		int top = 0;
		
		gl.glPushMatrix();
		gl.glRotated(90, 1, 0, 0);
//		gl.glTranslated(0, 10, 0);
		// court width 15m, height 14m
//		s *= scale;
		gl.glScaled(COURT_SCALE, COURT_SCALE, COURT_SCALE);
		gl.glTranslated(-courtSize.x/2, 0, 0);
		
		gl.glDisable(GL.GL_DEPTH_TEST);
		gl.glEnable(GL.GL_TEXTURE_2D);
		gl.glBindTexture(GL.GL_TEXTURE_2D, 13);
		gl.glBegin(GL.GL_POLYGON);
		gl.glTexCoord2d(0, 0);
		gl.glVertex2d(left, top);
		gl.glTexCoord2d(1, 0);
		gl.glVertex2d(left + w, top);
		gl.glTexCoord2d(1, 1);
		gl.glVertex2d(left + w, top + h);
		gl.glTexCoord2d(0, 1);
		gl.glVertex2d(left, top + h);
		gl.glEnd();
		gl.glDisable(GL.GL_TEXTURE_2D);
		gl.glFlush();
		gl.glEnable(GL.GL_DEPTH_TEST);
		
		gl.glPopMatrix();
	}
	
	
}
