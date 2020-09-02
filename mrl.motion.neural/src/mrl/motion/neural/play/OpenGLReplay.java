package mrl.motion.neural.play;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;

import javax.imageio.ImageIO;
import javax.media.opengl.GL;
import javax.media.opengl.glu.GLU;

import mrl.motion.viewer.GLDrawListener;
import mrl.motion.viewer.SWTViewableCanvas;

import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.widgets.Display;

import com.sun.opengl.util.GLUT;

public abstract class OpenGLReplay implements GLDrawListener{

	protected abstract SWTViewableCanvas getCanvas();
	
	protected abstract boolean update(int frame);
	
	private ByteBuffer buffer;
	
	private int frame;
	private String path;
	private boolean isSaved = true;
	
	public void capture(String path){
		this.path = path;
		SWTViewableCanvas canvas = getCanvas();
		frame = -1;
		canvas.setSize(1280, 720);
		canvas.addPostDrawListener(this);
		Display display = canvas.getDisplay();
		while (true){
			frame++;
			
			if (update(frame)) break;
			
			isSaved = false;
			while (true){
				display.readAndDispatch();
				if (isSaved) break;
			}
		}
		canvas.removePostDrawListener(this);
		System.out.println("Finished!");
	}
	
	@Override
	public void onDraw(GL gl, GLU glu, GLUT glut) {
		isSaved = true;
		Point size = getCanvas().getCanvas().getSize();
		BufferedImage image = makeScreenshot(gl, size.x, size.y);
		try {
            ImageIO.write(image, "png", new File(path + "_" + String.format("%04d", frame) + ".png"));
        } catch (IOException ex) {
        	throw new RuntimeException(ex);
        }
	}
	
	
	public BufferedImage makeScreenshot(GL gl, int width, int height) {
	    BufferedImage screenshot = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
	    Graphics graphics = screenshot.getGraphics();
	    if (buffer == null){
	    	buffer = ByteBuffer.allocate(width * height * 3);
	    }
	    buffer.clear();
//	    buffer = BufferUtils.createByteBuffer(width * height * 3);

	    gl.glReadPixels(0, 0, width, height, GL.GL_RGB, GL.GL_BYTE, buffer);


	    for (int h = 0; h < height; h++) {
	        for (int w = 0; w < width; w++) {
	            // The color are the three consecutive bytes, it's like referencing
	            // to the next consecutive array elements, so we got red, green, blue..
	            // red, green, blue, and so on..
	            graphics.setColor(new Color( buffer.get()*2, buffer.get()*2, buffer.get()*2 ));
	            graphics.drawRect(w,height - h, 1, 1); // height - h is for flipping the image
	        }
	    }
	    return screenshot;
	}
	
}
