package mrl.motion.neural.basket.sketch;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

public class SketchDataSerializer {

	public static void save(SketchData sketch, File file){
		try {
			ObjectOutputStream os = new ObjectOutputStream(new BufferedOutputStream(new FileOutputStream(file)));
			int version = 1;
			os.writeInt(version);
			
			HashMap<SketchNode, Integer> nodeMap = new HashMap<SketchNode, Integer>();
			ArrayList<SketchNode> nodeList = sketch.getNodeList();
			os.writeInt(nodeList.size());
			for (SketchNode node : nodeList){
				os.writeObject(node.label);
				os.writeInt(node.team);
				os.writeBoolean(node.isShoot);
				os.writeDouble(node.position.x);
				os.writeDouble(node.position.y);
				nodeMap.put(node, nodeMap.size());
			}
			
			ArrayList<SketchLine> lineList = sketch.getLineList();
			os.writeInt(lineList.size());
			for (SketchLine line : lineList){
				os.writeInt(nodeMap.get(line.source));
				os.writeInt(nodeMap.get(line.target));
				os.writeInt(line.type);
				
				os.writeDouble(line.lineVector.x);
				os.writeDouble(line.lineVector.y);
				os.writeInt(line.points.size());
				for (Point2d p : line.points){
					os.writeDouble(p.x);
					os.writeDouble(p.y);
				}
			}
			os.close();
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
	}
	
	public static SketchData load(File file){
		try {
			ObjectInputStream oi = new ObjectInputStream(new BufferedInputStream(new FileInputStream(file)));
			SketchData sketch = new SketchData();
			/*int version = */oi.readInt();
			
			int nodeSize = oi.readInt();
			ArrayList<SketchNode> nodeList = new ArrayList<SketchNode>();
			for (int i = 0; i < nodeSize; i++) {
				SketchNode node = sketch.newNode((String)oi.readObject());
				node.team = oi.readInt();
				node.isShoot = oi.readBoolean();
				Point2d position = new Point2d(oi.readDouble(), oi.readDouble());
				for (int j = 0; j < i; j++) {
					if (position.distance(nodeList.get(j).position) < SketchCanvas.NODE_RADIUS/2){
						position.set(nodeList.get(j).position);
					}
				}
				node.position = position;
				nodeList.add(node);
			}
			
			int lineSize = oi.readInt();
			for (int i = 0; i < lineSize; i++) {
				SketchLine line = sketch.newLine();
				line.source = nodeList.get(oi.readInt());
				line.target = nodeList.get(oi.readInt());
				line.type = oi.readInt();
				line.lineVector = new Vector2d(oi.readDouble(), oi.readDouble());
				
				
				int pointSize = oi.readInt();
				for (int j = 0; j < pointSize; j++) {
					line.points.add(new Point2d(oi.readDouble(), oi.readDouble()));
				}
			}
			System.out.println("load : " + nodeSize + " , " + lineSize);
			
			oi.close();
			return sketch;
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
	}
}
