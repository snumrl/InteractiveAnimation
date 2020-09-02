package mrl.motion.neural.basket.sketch;

import static mrl.util.MathUtil.round;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mrl.motion.neural.basket.sketch.SketchLine.LineType;
import mrl.widget.Rectangle2d;
import mrl.widget.ScalableCanvas2D;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.Font;
import org.eclipse.swt.graphics.FontData;
import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.ImageData;
import org.eclipse.swt.graphics.ImageLoader;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.widgets.Composite;

public class SketchCanvas extends ScalableCanvas2D {
	
	public static final Point2d BASKET_POS = new Point2d(0, 122);
	public static final double NODE_RADIUS = 30;
	
	private SketchData data;
	private SketchNode selectedItem = null;
	private Point2d lastMousePoint = null;
	
	private enum State { AddNode, DrawLine };
	private State state;
	private SketchLine drawingLine = null;
	private SketchNode addNode = null;
	private int drawLineType = -1;
	private boolean isShoot = false;
	private boolean isDelete = false;
	
	private Font font;
	private Image court;
	
	private Color[][] personColors;
	
	// for screen shot
	public int shotIndex = 0;
	public boolean shotEnabled = false;
	
	public boolean showLegend = false;
	
	public SketchCanvas(Composite parent) {
		super(parent);
		
		FontData fontData = new FontData();
		fontData.setName("Segoe UI");
		fontData.height = 30;
		font = new Font(getDisplay(), fontData);
		
		court = new Image(getDisplay(), SketchCanvas.class.getResourceAsStream("field_map.jpg"));
		//court = new Image(getDisplay(), SketchCanvas.class.getResourceAsStream("court.png"));
		ImageData courtData = court.getImageData();
		courtData.alpha = 64;
		court = new Image(getDisplay(), courtData);
		
		//center = new Point2d(0, court.getBounds().height/2);
		center = new Point2d(0, 520);
		
		int[][][] pColors = new int[][][]{
				// attacker
				{
					{255,203,203},
					{227,128,128},
					{170,57,57},
					{255,84, 100},
					{255,105,85},
					{248,83,133},
					{ 255, 136, 136 },
					{ 241, 158, 183 },
					{ 255, 165, 137 },
					{ 205, 126, 126 },
					{ 230, 180, 180 },
				},
				// defender
				{
					{67,47,117},
					{112,95,155},
					{175,165,201},
					{94,136,233},
					{93, 142, 233},
					{88, 166, 231},
					{ 127, 117, 195 },
					{ 50, 162, 183},
					{ 94, 143, 185 },
					{ 98, 95, 143 }, 
					{ 132, 130, 160 },
				},
		};
		personColors = new Color[pColors.length][];
		for (int i = 0; i < pColors.length; i++) {
			int[][] c = pColors[i];
			personColors[i] = new Color[c.length];
			for (int j = 0; j < c.length; j++) {
				personColors[i][j] = new Color(getDisplay(), c[j][0], c[j][1], c[j][2]);
			}
		}
	}
	
	private Color getColor(String label){
		int teamIdx = label.startsWith(SketchNode.TEAM_ATTACK_PREFIX) ? 0 : 1;
		int pIdx = Integer.parseInt(label.substring(1, 2))-1;
		return personColors[teamIdx][pIdx % personColors[teamIdx].length];
	}
	

	public void setSketchData(SketchData data) {
		this.data = data;
		redrawCanvas();
	}
	public SketchData getSketchData() {
		return data;
	}
	
	
	
	public void addNode(SketchNode node){
		this.addNode = node;
		state = State.AddNode;
	}
	
	public void drawLine(int type){
		drawLineType = type;
		state = State.DrawLine;
	}
	
	public Point getCourtSize(){
		return new Point(court.getBounds().width, court.getBounds().height);
	}
	
	public void setShoot(boolean isShoot) {
		this.isShoot = isShoot;
	}

	public void setDelete(boolean isDelete) {
		this.isDelete = isDelete;
	}

	@Override
	protected void drawContents(GC gc) {
		if (data == null) return;
		
		gc.drawImage(court, -court.getBounds().width/2, (int)(-180*0.7));
//		gc.drawImage(court, -court.getBounds().width/2, 0);
//		gc.drawImage(court, -court.getBounds().width/2, court.getBounds().height/2);
		
		gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_WHITE));
		gc.setLineWidth(8);
		// °ñ´ë À§Ä¡ 145¿¡¼­ 122·Î ¹Ù²Þ
		int gOffset = (int)BASKET_POS.y;
		gOffset = (int)(gOffset*0.7);
		drawLine(gc, 0, gOffset - 40, 0, gOffset - 20);
		drawLine(gc, -60, gOffset - 40, 60, gOffset - 40);
		gc.drawOval(-20, gOffset - 20, 40, 40);

		saveAsImage(gc);
		
		
		gc.setLineWidth(4);
		for (SketchLine line : data.getLineList()){
			gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_BLACK));
			LineType type = line.type();
			if (type == LineType.Pass) {
			//if (type == LineType.Pass || type == LineType.Dribble){
				gc.setLineDash(new int[]{ 10, 5 });
			} else if (type == LineType.Run){
				gc.setLineDash(null);
				gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_DARK_GRAY));
			} else {
				gc.setLineDash(null);
			}
			ArrayList<Point2d> points = line.getPoints();
			for (int i = 0; i < points.size() - 1; i++) {
				Point2d p1 = points.get(i);
				Point2d p2 = points.get(i+1);
				if (i >= points.size() - 4){
					gc.setLineDash(null);
				}
				drawLine(gc, p1.x, p1.y, p2.x, p2.y);
			}
			gc.setLineDash(null);
			
			saveAsImage(gc);
		}
		
		for (SketchNode node : data.getNodeList()){
			Point2d p = node.position;
			double radius = NODE_RADIUS;

			if(node.isShoot) {
				gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_BLACK));
				gc.setBackground(getDisplay().getSystemColor(SWT.COLOR_BLACK));
				
				double dx = 0 - p.x;
				double dy = gOffset - p.y;
				Vector2d direction = new Vector2d(dx, dy);
				direction.normalize();
				
				Vector2d offset = new Vector2d(-direction.y, direction.x);
				offset.normalize();
				offset.scale(radius*0.8);
				
				Point2d pp = new Point2d(p);
				double move = radius*0.4;
				pp.x += direction.x*move;
				pp.y += direction.y*move;
				
				direction.scale(radius*1.3);
				gc.fillPolygon(new int[]{
						round(pp.x + offset.x), round(pp.y + offset.y), 
						round(pp.x - offset.x), round(pp.y - offset.y), 
						round(pp.x + direction.x), round(pp.y + direction.y), 
				});
			}
			
			gc.setBackground(getColor(node.label));
			gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_BLACK));
			
			fillOval(gc, p.x - radius, p.y - radius, radius*2, radius*2);
			drawOval(gc, p.x - radius, p.y - radius, radius*2, radius*2);
			
			gc.setFont(font);
			Point extent = gc.textExtent(node.label);
			if (node.team == 0){
				gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_BLACK));
			} else {
				gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_WHITE));
			}
			gc.drawString(node.label, (int)(p.x - extent.x/2), (int)(p.y - extent.y/2), true);
			
			saveAsImage(gc);
		}

		if (showLegend){
			drawLegend(gc);
		}

		saveAsImage(gc);
		shotEnabled = false;
	}
	
	private void drawLegend(GC gc) {
		Point2d offset = new Point2d(-600, 800);
		
		gc.setForeground(getDisplay().getSystemColor(SWT.COLOR_BLACK));
		gc.setBackground(getDisplay().getSystemColor(SWT.COLOR_BLACK));

		gc.drawOval(round(offset.x + 160 - NODE_RADIUS), round(offset.y - 60 - NODE_RADIUS), 
				(int)NODE_RADIUS * 2, (int)NODE_RADIUS * 2);
		
		gc.fillPolygon(new int[]{
				round(offset.x+150), round(offset.y - NODE_RADIUS *.8),
				round(offset.x+150), round(offset.y + NODE_RADIUS *.8),
				round(offset.x+150 + NODE_RADIUS * 1.3), round(offset.y)
				});
		
		gc.setLineDash(new int[]{ 10, 5 });
		drawLine(gc, offset.x + 100, offset.y+50, offset.x+200, offset.y+50);
		
		gc.setLineDash(null);
		drawLine(gc, offset.x + 100, offset.y+100, offset.x+200, offset.y+100);
		
		drawLine(gc, offset.x + 100, offset.y+150, offset.x+105, offset.y+180);
		drawLine(gc, offset.x + 105, offset.y+180, offset.x+115, offset.y+120);
		drawLine(gc, offset.x + 115, offset.y+120, offset.x+120, offset.y+150);
		drawLine(gc, offset.x + 120, offset.y+150, offset.x+200, offset.y+150);
		
		drawLine(gc, offset.x + 100, offset.y+200, offset.x+200, offset.y+200);
		drawLine(gc, offset.x + 200, offset.y+180, offset.x+200, offset.y+220);
		
		double dx = 0.93969262078590838405410927732473 * NODE_RADIUS;
		double dy = 0.34202014332566873304409961468226 * NODE_RADIUS;
		
		for(int i = 0; i < 3; ++i) {
			drawLine(gc, offset.x+200-dx, offset.y+50-dy+i*50, offset.x+200, offset.y+50+i*50);
			drawLine(gc, offset.x+200-dx, offset.y+50+dy+i*50, offset.x+200, offset.y+50+i*50);
		}
		
		gc.setFont(font);
		String[] legends = {"locate", "shoot a ball", "pass a ball", "run, walk, or dribble", "feint move", "set a screen"};
		for(int i = 0; i < legends.length; ++i) {
			gc.drawString(legends[i], round(offset.x+220), round(offset.y+i*50-82 - (i == 0 ? 10 : 0)), true);
		}
	}
	
	private SketchNode getNodeByPosition(Point2d p){
		for (SketchNode node : data.getNodeList()){
			if (node.position.distance(p) < NODE_RADIUS){
				return node;
			}
		}
		return null;
	}
	
	private SketchLine getLineByPosition(Point2d p){
		for (SketchLine line : data.getLineList()){
			ArrayList<Point2d> points = line.getPoints();
			for (int i = 0; i < points.size()-1; i++) {
				Point2d p1 = points.get(i);
				Point2d p2 = points.get(i+1);
				
				Vector2d v1 = new Vector2d();
				v1.sub(p2, p1);
				Vector2d v2 = new Vector2d();
				v2.sub(p, p1);
				
				v1.normalize();
				double dot = v1.dot(v2);
				if (dot > 0 && dot < p1.distance(p2)){
					v1.scale(dot);
					Point2d pp = new Point2d();
					pp.add(p1, v1);
					double d = pp.distance(p);
					if (d < NODE_RADIUS/2){
						return line;
					}
				}
			}
		}
		return null;
	}

	public void saveAsImage(GC ogc) {
		if(!shotEnabled) return; 
		++shotIndex;
		if(shotIndex > 100) { shotEnabled = false; return; }
		
		System.out.println(shotIndex);
		
		Image image = new Image(getDisplay(), court.getBounds().width, ogc.getClipping().height);
		ogc.copyArea(image, (ogc.getClipping().width - image.getBounds().width)/2, 0);
		
		ImageData imageData = image.getImageData();
		image.dispose();
		
		ImageLoader loader = new ImageLoader();
		loader.data = new ImageData[] {imageData};
		loader.save("shots\\shot" + shotIndex + ".png", SWT.IMAGE_PNG);
		
		ogc.setBackground(getDisplay().getSystemColor(SWT.COLOR_WHITE));
		ogc.fillRectangle(-5000, -5000, 10000, 10000);
	}
	
	@Override
	protected boolean onMouseDown(Point2d p, MouseEvent e) {
		System.out.println("sketch mouse down : " + p);
		if (state == State.AddNode){
			SketchNode node = data.newNode(addNode.label);
			node.team = addNode.team;
			node.position = new Point2d(p);
			state = null;
			return true;
		}
		
		lastMousePoint = null;
		if (isDelete){
			SketchNode node = getNodeByPosition(p);
			if (node != null){
				data.deleteNode(node);
			} else {
				SketchLine line = getLineByPosition(p);
				if (line != null){
					data.deleteLine(line);
				}
			}
			isDelete = false;
		} else {
			selectedItem = getNodeByPosition(p);
			if (selectedItem != null){
				if (isShoot){
					selectedItem.isShoot = !selectedItem.isShoot;
					isShoot = false;
				} else if (state == State.DrawLine){
					drawingLine = data.newLine();
					drawingLine.source = selectedItem;
					drawingLine.type = drawLineType;
					drawingLine.addPoint(new Point2d(selectedItem.position));
					
					if(drawingLine.type() == LineType.Feint) {
						/*
						Vector2d basketDir = new Vector2d(selectedItem.position.x, selectedItem.position.y-145);
						basketDir.scale(-40 / basketDir.length());
						Point2d basket = new Point2d(selectedItem.position);
						basket.add(basketDir);

						drawingLine.addPoint(basket);
						p = basket;
						*/						
					}
				}
				
				lastMousePoint = p;
				return true;
			}
		}
		return true;
	}

	@Override
	protected boolean onMouseMove(Point2d p, MouseEvent e) {
		if (selectedItem == null) return false;
		
		if (state == State.DrawLine){
			drawingLine.addPoint(p);
		} else {
			Point2d diff = new Point2d();
			diff.sub(p, lastMousePoint);
			selectedItem.position.x += diff.x;
			selectedItem.position.y += diff.y;
			lastMousePoint = p;
		}
		
		return true;
	}

	@Override
	protected boolean onMouseUp(Point2d p, MouseEvent e) {
		if (selectedItem == null) return false;
		
		if (state == State.DrawLine){
			drawingLine.target = getNodeByPosition(p);
			if (drawingLine.target == null){
				if (drawingLine.type() == LineType.Dribble || drawingLine.type() == LineType.Run || drawingLine.type() == LineType.Feint){
					SketchNode source = drawingLine.source;
					SketchNode target = data.newNode(source.label);
					target.team = source.team;
					target.position = new Point2d(p);
					drawingLine.target = target;
				}
			}
			
			if (drawingLine.target == null){
				data.removeLine(drawingLine);
			} else {
				drawingLine.fix();
			}
			drawingLine = null;
		}
		selectedItem = null;
		lastMousePoint = null;
		state = null;
		addNode = null;
		
		return true;
	}

	@Override
	protected boolean onMouseDoubleClick(Point2d p, MouseEvent e) {
		return true;
	}

	@Override
	protected boolean onMouseScrolled(Point2d p, MouseEvent e) {
		return true;
	}

	@Override
	protected Rectangle getContentBoundary() {
		return null;
	}

	@Override
	protected void onBoundarySelection(Rectangle2d boundary, MouseEvent e) {
		
	}
}
