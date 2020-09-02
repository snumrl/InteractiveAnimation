package mrl.motion.neural.basket.sketch;

import mrl.motion.neural.basket.sketch.SketchLine.LineType;
import mrl.widget.ShellOpener;

import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Font;
import org.eclipse.swt.graphics.FontData;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;

public class SketchDrawer extends Composite{
	
	public SketchCanvas canvas;
	private Font font;

	public SketchDrawer(Composite parent) {
		super(parent, SWT.NONE);
		
		this.setLayout(layout(3));
		
		FontData fontData = new FontData();
		fontData.setName("Segoe UI");
		fontData.height = 20;
		font = new Font(getDisplay(), fontData);
		
		createMenuBar();
		
		createPlayerBar(0, "Attack");
		canvas = new SketchCanvas(this);
		createPlayerBar(1, "Defence");
		
		canvas.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
	}
	
	public SketchCanvas getCanvas() {
		return canvas;
	}

	private GridLayout layout(int columns){
		GridLayout l = new GridLayout(columns, false);
		l.marginWidth = l.marginHeight = 0;
		return l;
	}
	
	public SketchData getSketchData() {
		return canvas.getSketchData();
	}

	public void setSketchData(SketchData sketchData) {
		canvas.setSketchData(sketchData);
	}
	
	private Button button(Composite parent, String label, Listener listener){
		Button button = new Button(parent, SWT.PUSH);
		button.setText(label);
		button.setLayoutData(new GridData(SWT.FILL, SWT.FILL, false, false));
		button.addListener(SWT.Selection, listener);
		button.setFont(font);
		return button;
	}

	private void createMenuBar(){
		Composite menuComp = new Composite(this, SWT.NONE);
		menuComp.setLayoutData(new GridData(SWT.CENTER, SWT.FILL, true, false, 3, 1));
		
		LineType[] menuList = LineType.values();
		menuComp.setLayout(layout(menuList.length + 2));
		for (int i = 0; i < menuList.length; i++) {
			final int type = i;
			button(menuComp, menuList[i].toString(), new Listener() {
				@Override
				public void handleEvent(Event event) {
					canvas.drawLine(type);
				}
			});
		}
		
		button(menuComp, "Shoot", new Listener() {
			@Override
			public void handleEvent(Event event) {
				canvas.setShoot(true);
			}
		});
		
		button(menuComp, "Delete", new Listener() {
			@Override
			public void handleEvent(Event event) {
				canvas.setDelete(true);
			}
		});
	}
	

	private void createPlayerBar(final int team, String teamName){
		Composite composite = new Composite(this, SWT.NONE);
		composite.setLayoutData(new GridData(SWT.FILL, SWT.CENTER, false, true));
		composite.setLayout(layout(1));
		
		Label teamLabel = new Label(composite, SWT.NONE);
		teamLabel.setLayoutData(new GridData(SWT.FILL, SWT.FILL, false, false));
		teamLabel.setText(teamName);
		teamLabel.setFont(font);
		
		int memberSize = 5;
		for (int i = 0; i < memberSize; i++) {
			final String label = teamName.substring(0, 1) + (i+1);
			button(composite, label, new Listener() {
				@Override
				public void handleEvent(Event event) {
					SketchNode newNode = new SketchNode();
					newNode.label = label;
					newNode.team = team;
					canvas.addNode(newNode);
				}
			});
			
		}
	}
	
	public static void main(String[] args) {
		ShellOpener.open(new ShellOpener() {
			@Override
			public void createComposite(Shell shell) {
				SketchDrawer drawer = new SketchDrawer(shell);
				drawer.setSketchData(new SketchData());
				shell.setSize(1500, 1250);
			}
		});
	}
}
