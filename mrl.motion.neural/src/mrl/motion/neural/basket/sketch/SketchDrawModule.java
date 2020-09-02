package mrl.motion.neural.basket.sketch;

import java.io.File;

import mrl.widget.app.MainApplication.WindowPosition;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.FileDialog;

public class SketchDrawModule extends Module{

	private SketchDrawer sketchDrawer;
	
	@Override
	protected void initializeImpl() {
		
		sketchDrawer = addWindow(new SketchDrawer(dummyParent()), WindowPosition.Main);
		sketchDrawer.setSketchData(new SketchData());
		
		addMenu("&Menu", "&Load As..\tCtrl+L", SWT.MOD1 + 'l', new Runnable() {
			@Override
			public void run() {
				FileDialog d = new FileDialog(sketchDrawer.getShell(), SWT.OPEN);
				String[] exts = new String[]{ "*.sk" };
				d.setFilterExtensions(exts);
				d.setFilterNames(exts);
				String path = d.open();
				if (path == null) return;
				loadSketch(path);
			}
		});
		
		addMenu("&Menu", "Save As..\tCtrl+S", SWT.MOD1 + 's', new Runnable() {
			@Override
			public void run() {
				FileDialog d = new FileDialog(sketchDrawer.getShell(), SWT.SAVE);
				String[] exts = new String[]{ "*.sk" };
				d.setFilterExtensions(exts);
				d.setFilterNames(exts);
				String path = d.open();
				if (path == null) return;
				
				SketchDataSerializer.save(sketchDrawer.getSketchData(), new File(path));
			}
		});
	}
	
	public void loadSketch(String file){
		sketchDrawer.setSketchData(SketchDataSerializer.load(new File(file)));
	}

	public SketchDrawer getSketchDrawer() {
		return sketchDrawer;
	}

	public static void main(String[] args) {
		MainApplication.run(new SketchDrawModule());
	}
}
