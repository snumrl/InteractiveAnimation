package mrl.motion.neural.run;

import java.util.ArrayList;

import javax.vecmath.Point2d;

import mrl.util.FileUtil;
import mrl.util.Utils;
import mrl.widget.app.MainApplication.WindowPosition;
import mrl.widget.app.Module;
import mrl.widget.graph.DrawGraph;
import mrl.widget.graph.DrawLink;
import mrl.widget.graph.DrawNode;
import mrl.widget.graph.GraphViewer;

import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.FileDialog;

public class PathTrajectoryModule extends Module{

	public static int TRAJECOTRY_SIZE = 20;
	private GraphViewer viewer;
	private DrawGraph graph;

	@Override
	protected void initializeImpl() {
		viewer = addWindow(new GraphViewer(dummyParent()), WindowPosition.Right);
		
		graph = new DrawGraph();
		for (int i = 0; i < TRAJECOTRY_SIZE; i++) {
			DrawNode node = new DrawNode();
			node.position = new Point2d(i*100, 0);
			graph.nodeList.add(node);
			if (i > 0){
				DrawLink link = new DrawLink();
				link.source = graph.nodeList.get(graph.nodeList.size()-2);
				link.target = graph.nodeList.get(graph.nodeList.size()-1);
				graph.linkList.add(link);
			}
		}
		graph.setInitialized(true);
		viewer.setGraph(graph);
		viewer.fitToScreen();
		
		addMenu("&Path Trajectory", "&Load\tCtrl+L", SWT.MOD1 + 'L', new Runnable() {
			@Override
			public void run() {
				FileDialog d = new FileDialog(viewer.getShell(), SWT.OPEN);
				String[] exts = new String[]{ "*.path" };
				d.setFilterExtensions(exts);
				d.setFilterNames(exts);
				String path = d.open();
				if (path == null) return;
				loadTrajectory(path);
			}
		});
		addMenu("&Path Trajectory", "&Save\tCtrl+S", SWT.MOD1 + 'S', new Runnable() {
			@Override
			public void run() {
				FileDialog d = new FileDialog(viewer.getShell(), SWT.SAVE);
				String[] exts = new String[]{ "*.path" };
				d.setFilterExtensions(exts);
				d.setFilterNames(exts);
				String path = d.open();
				if (path == null) return;
				ArrayList<Point2d> list = new ArrayList<Point2d>();
				for (DrawNode node : graph.nodeList){
					list.add(node.position);
				}
				FileUtil.writeObject(list, path);
			}
		});
		addMenu("&Path Trajectory", "&Extend Path\tCtrl+E", SWT.MOD1 + 'E', new Runnable() {
			@Override
			public void run() {
				Point2d p = Utils.last(graph.nodeList).position;
				for (int i = 0; i < 10; i++) {
					DrawNode node = new DrawNode();
					node.position = new Point2d(i*100, 0);
					node.position.add(p);
					graph.nodeList.add(node);
					
					DrawLink link = new DrawLink();
					link.source = graph.nodeList.get(graph.nodeList.size()-2);
					link.target = graph.nodeList.get(graph.nodeList.size()-1);
					graph.linkList.add(link);
				}
			}
		});
	}
	
	public void loadTrajectory(String file){
		@SuppressWarnings("unchecked")
		ArrayList<Point2d> list = (ArrayList<Point2d>)FileUtil.readObject(file);
		for (int i = 0; i < list.size(); i++) {
			if (i < graph.nodeList.size()){
				graph.nodeList.get(i).position.set(list.get(i));
			} else {
				DrawNode node = new DrawNode();
				node.position = new Point2d(list.get(i));
				graph.nodeList.add(node);
				if (i > 0){
					DrawLink link = new DrawLink();
					link.source = graph.nodeList.get(graph.nodeList.size()-2);
					link.target = graph.nodeList.get(graph.nodeList.size()-1);
					graph.linkList.add(link);
				}
			}
		}
		viewer.redrawCanvas();
	}
	
	public ArrayList<Point2d> getTrajectory(){
		ArrayList<Point2d> list = new ArrayList<Point2d>();
		for (DrawNode node : graph.nodeList){
			list.add(new Point2d(node.position));
		}
		return list;
	}

	public GraphViewer getViewer() {
		return viewer;
	}

}
