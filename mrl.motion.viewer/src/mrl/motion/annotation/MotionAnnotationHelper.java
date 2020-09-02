package mrl.motion.annotation;

import java.io.File;
import java.util.ArrayList;

import javax.vecmath.Vector3d;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.MotionAnnotation;
import mrl.motion.data.MotionAnnotation.MotionAnnValueGetter;
import mrl.motion.data.MotionAnnotationManager;
import mrl.motion.data.MotionData;
import mrl.motion.data.MultiCharacterFolder;
import mrl.motion.data.MultiCharacterFolder.MultiCharacterFiles;
import mrl.motion.viewer.BaseApplication;
import mrl.motion.viewer.MotionIntervalSelector;
import mrl.motion.viewer.MotionNavigator;
import mrl.motion.viewer.MotionViewer;
import mrl.motion.viewer.MotionListViewer.MotionListGetter;
import mrl.motion.viewer.MultiCharacterBVHListViewer.MultiFileMotionGetter;
import mrl.motion.viewer.MultiCharacterNavigator;
import mrl.util.Configuration;
import mrl.util.ObjectSerializer;
import mrl.widget.ObjectPropertyPanel;
import mrl.widget.dockable.DockableTabFolder;
import mrl.widget.dockable.SashFormContainer;
import mrl.widget.dockable.SashFormContainer.DockingPosition;
import mrl.widget.table.FilterUtil;
import mrl.widget.table.FilterableTable;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.DisposeEvent;
import org.eclipse.swt.events.DisposeListener;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Menu;
import org.eclipse.swt.widgets.MenuItem;

public class MotionAnnotationHelper extends BaseApplication{
	
	public static String[] BALL_CONTACTS = { "b_left", "b_right" };
	
	private String dataFolder;
	private String outputFolder;
	private List fileList;
	private ArrayList<MotionListGetter> motionGetterList;
	protected MotionIntervalSelector motionViewer;
	protected MotionAnnotationTimeline timeline;
	private FilterableTable<MotionAnnotation> annotationTable;
	
	private ArrayList<MotionAnnotation> annotationList;
	
	private ObjectPropertyPanel propertyPanel;

	protected File currentAnnotationFile;
	protected ArrayList<File> currentFileList;
	
	private boolean saveFile = true;
	
	private Vector3d[] skeletonColors;
	
	protected Menu submenu;
	
	public MotionAnnotationHelper(String dataFolder, String outputFolder){
		this.dataFolder = dataFolder;
		this.outputFolder = outputFolder;
		
		if (new File(outputFolder).exists() == false){
			new File(outputFolder).mkdirs();
		}
	}
	
	public MultiCharacterNavigator getNavigator(){
		return motionViewer.getNavigator();
	}
	
	public MotionAnnotationTimeline getTimeline(){
		return timeline;
	}
	
	public static boolean isBallContact(String type){
		for (String b : BALL_CONTACTS){
			if (b.equals(type)) return true; 
		}
		return false;
	}

	@Override
	protected void createInitialUI() {
		DockableTabFolder leftFolder = new DockableTabFolder(container, SWT.BORDER);
		container.dropInitialControl(leftFolder);
		fileList = new List(leftFolder, SWT.BORDER | SWT.V_SCROLL | SWT.H_SCROLL);
		leftFolder.addItem("File List", fileList, false);
		
		DockableTabFolder rightFolder = new DockableTabFolder(container, SWT.BORDER);
		container.dropNewControl(rightFolder, DockingPosition.Right, null);
		motionViewer = new MotionIntervalSelector(rightFolder);
		rightFolder.addItem("Motion", motionViewer, false);
		
		
		SashFormContainer rightContainer = (SashFormContainer)rightFolder.getParent(); 
		DockableTabFolder timelineFolder = new DockableTabFolder(rightContainer, SWT.BORDER);
		rightContainer.dropNewControl(timelineFolder, DockingPosition.Bottom, null);
		timeline = new MotionAnnotationTimeline(timelineFolder);
		timelineFolder.addItem("Timeline", timeline, false);
		
		SashFormContainer leftContainer = (SashFormContainer)leftFolder.getParent(); 
		DockableTabFolder propertyFolder = new DockableTabFolder(leftContainer, SWT.BORDER);
		leftContainer.dropNewControl(propertyFolder, DockingPosition.Bottom, null);
		propertyPanel = new ObjectPropertyPanel(propertyFolder, MotionAnnotation.class);
		propertyFolder.addItem("Label Property", propertyPanel, false);
		
		container.getSashForm().setWeights(new int[]{15, 85});
		rightContainer.getSashForm().setWeights(new int[]{80, 20});
		leftContainer.getSashForm().setWeights(new int[]{45, 55});
		
		SashFormContainer middleContainer = (SashFormContainer)rightFolder.getParent().getParent().getParent(); 
		DockableTabFolder tableFolder = new DockableTabFolder(middleContainer, SWT.BORDER);
		middleContainer.dropNewControl(tableFolder, DockingPosition.Right, null);
		annotationTable = new FilterableTable<MotionAnnotation>(tableFolder, false);
		tableFolder.addItem("Label List", annotationTable, false);
//		annotationTable
		
		
		annotationTable.setFilterableFields("type", 
				"subtype", "active part", 
				"passive part", "inter type", "include", "multi-iterac");
		middleContainer.getSashForm().setWeights(new int[]{70, 30});
		
		skeletonColors = new Vector3d[4];
		skeletonColors[0] = new Vector3d(1, 0.8, 0.7);
		skeletonColors[1] = new Vector3d(0.7, 1, 0.8);
		skeletonColors[2] = new Vector3d(0.6, 0.6, 1);
		skeletonColors[3] = new Vector3d(0.8, 0.8, 0.8);
		
		final Vector3d[] headColors = new Vector3d[4];
		
		
		propertyPanel.addDisableFields("file", "person");
		setSampleValues();
		
		
		motionViewer.getNavigator().setGoStartAfterPlay(false);
		motionViewer.getNavigator().setStartFromZero(true);
		
		
		motionViewer.getNavigator().getSlider().addListener(SWT.Selection, new Listener() {
			@Override
			public void handleEvent(Event event) {
				int motionIndex = motionViewer.getNavigator().getMotionIndex();
				timeline.setMotionIndex(motionIndex);
			}
		});
		
		timeline.addListener(SWT.Modify, new Listener() {
			@Override
			public void handleEvent(Event event) {
				int motionIndex = motionViewer.getNavigator().getMotionIndex();
				for (int i = 0; i < headColors.length; i++) {
					headColors[i] = timeline.getPersonColor(i + 1, motionIndex);
				}
				motionViewer.getNavigator().getViewer().setHeadColorList(headColors);
			}
		});
		
		propertyPanel.addListener(SWT.Modify, new Listener() {
			@Override
			public void handleEvent(Event event) {
				timeline.callRedraw();
			}
		});
		
		timeline.addListener(SWT.Selection, new Listener() {
			@Override
			public void handleEvent(Event event) {
				MotionAnnotation annotation = timeline.getSelectedAnnotation();
				propertyPanel.setObject(annotation);
				
				if (timeline.getOnMouseAnnotation() != null){
					annotation = timeline.getOnMouseAnnotation();
					MultiCharacterNavigator navigator = motionViewer.getNavigator();
					if (annotation.endFrame < navigator.getMinSelection() - 1 || annotation.startFrame > navigator.getMaxSelection() - 1){
						motionViewer.selectAll();
					}
					motionViewer.getNavigator().setMotionIndex(annotation.startFrame);
				}
				
				int person = timeline.getSelectedPerson();
				Vector3d[] colors = motionViewer.getNavigator().getColorList();
				for (int i = 0; i < skeletonColors.length; i++) {
					if (i == (person - 1)){
						colors[i].set(new Vector3d(1, 1, 0.4));
					} else {
						colors[i] = new Vector3d(skeletonColors[i]);
					}
				}
			}
		});
		timeline.addListener(SWT.MouseDoubleClick, new Listener() {
			@Override
			public void handleEvent(Event event) {
				MotionAnnotation annotation = timeline.getOnMouseAnnotation();
				MultiCharacterNavigator navigator = motionViewer.getNavigator();
				if (annotation != null){
					navigator.stopAnimation();
					motionViewer.setSelectionBound(annotation.startFrame+1, annotation.endFrame+1);
					navigator.setMotionIndex(annotation.startFrame);
					navigator.startAnimation();
				} else {
					int mIndex = navigator.getMotionIndex();
					motionViewer.selectAll();
					navigator.setMotionIndex(mIndex);
				}
			}
		});
		
		
		motionViewer.getNavigator().getViewer().getCanvas().addKeyListener(new KeyListener() {
			@Override
			public void keyReleased(KeyEvent e) {
			}
			@Override
			public void keyPressed(KeyEvent e) {
				onKeyDown(e);
			}
		});
		
		annotationTable.getTable().addListener(SWT.Selection, new Listener() {
			@Override
			public void handleEvent(Event event) {
				if (annotationTable.getTable().getSelection().length != 1) return;
				MotionAnnotation annotation = (MotionAnnotation)annotationTable.getTable().getSelection()[0].getData();
				
				int fileIndex = getFileIndex(annotation);
				if (fileList.getSelectionIndex() != fileIndex){
					fileList.select(fileIndex);
					fileList.notifyListeners(SWT.Selection, new Event());
				}
				
				for (MotionAnnotation ann : annotationList){
					if (ann.equals(annotation)){
						annotation = ann;
						break;
					}
				}
				
				timeline.setSelectedPerson(annotation.person);
				timeline.selectAnnotation(annotation);
				
				
				MultiCharacterNavigator navigator = motionViewer.getNavigator();
				navigator.stopAnimation();
				motionViewer.setSelectionBound(annotation.startFrame+1, annotation.endFrame+1);
				navigator.setMotionIndex(annotation.startFrame);
				navigator.startAnimation();
				
				propertyPanel.setFieldFocus("type");
			}
		});
		
		
		loadFileList();
		updateAnnotationTable();
		
		
		
		container.addDisposeListener(new DisposeListener() {
			@Override
			public void widgetDisposed(DisposeEvent e) {
				saveCurrentAnnotation();
			}
		});
	}
	
	private int getFileIndex(MotionAnnotation ann){
		for (int i = 0; i < motionGetterList.size(); i++) {
			MultiFileMotionGetter getter = ((MultiFileMotionGetter)motionGetterList.get(i));
			for (File file : getter.getFiles().fileList){
				if (file.getName().equals(ann.file)) return i;
			}
		}
		return -1;
	}
	
	private void updateAnnotationTable(){
		saveCurrentAnnotation();
		
		ArrayList<MotionAnnotation> totalList = new MotionAnnotationManager(outputFolder,false).getTotalAnnotations();
		ArrayList<MotionAnnotation> filteredList = new ArrayList<MotionAnnotation>();
		for (MotionAnnotation ann : totalList){
//			if (ann.startFrame == ann.endFrame) continue;
			if ("1".equals(ann.type)) continue;
			filteredList.add(ann);
		}
		annotationTable.setItemList(filteredList, new MotionAnnValueGetter());
	}
	
	private void loadFileList(){
		
		fileList.addListener(SWT.Selection, new Listener() {
			@Override
			public void handleEvent(Event event) {
				updateFileSelection();
			}
		});
		
		
		MultiCharacterFolder folder = MultiCharacterFolder.loadFolder(new File(dataFolder));
		motionGetterList = new ArrayList<MotionListGetter>();
		for (MultiCharacterFiles files : folder.list){
			motionGetterList.add(new MultiFileMotionGetter(files));
		}
		
		fileList.removeAll();
		String[] items = new String[motionGetterList.size()];
		for (int i = 0; i < items.length; i++) {
			items[i] = motionGetterList.get(i).getLabel();
		}
		fileList.setItems(items);
		fileList.select(0);
		fileList.notifyListeners(SWT.Selection, new Event());
		
//		motionListViewer.setMotionDataList(list);
	}
	
	private void onAnnPropertyChange(){
		propertyPanel.setObject(timeline.getSelectedAnnotation());
		timeline.callRedraw();
	}
	
	@Override
	protected void createMenu() {
		Menu bar = new Menu (shell, SWT.BAR);
		shell.setMenuBar (bar);
		MenuItem menuItem = new MenuItem (bar, SWT.CASCADE);
		menuItem.setText ("&Menu");
		submenu = new Menu (shell, SWT.DROP_DOWN);
		menuItem.setMenu (submenu);
		
		addMenu(submenu, "&New Label\tCtrl+N", SWT.MOD1 + 'N', new Runnable() {
			public void run() {
				newAnnotation();
			}
		});
		addMenu(submenu, "Set &start frame\tCtrl+S", SWT.MOD1 + 'S', new Runnable() {
			public void run() {
				MotionAnnotation annotation = timeline.getSelectedAnnotation();
				if (annotation == null) return;
				annotation.startFrame = getCurrentFrame();
				annotation.endFrame = Math.max(annotation.startFrame, annotation.endFrame);
				onAnnPropertyChange();
			}
		});
		addMenu(submenu, "Set &end frame\tCtrl+E", SWT.MOD1 + 'E', new Runnable() {
			public void run() {
				MotionAnnotation annotation = timeline.getSelectedAnnotation();
				if (annotation == null) return;
				annotation.endFrame = getCurrentFrame();
				annotation.startFrame = Math.min(annotation.startFrame, annotation.endFrame);
				onAnnPropertyChange();
			}
		});
		addMenu(submenu, "Set &interaction frame\tCtrl+I", SWT.MOD1 + 'I', new Runnable() {
			public void run() {
				MotionAnnotation annotation = timeline.getSelectedAnnotation();
				if (annotation == null) return;
				annotation.interactionFrame = getCurrentFrame();
				annotation.startFrame = Math.min(annotation.startFrame, annotation.interactionFrame);
				annotation.endFrame = Math.max(annotation.interactionFrame, annotation.endFrame);
				onAnnPropertyChange();
			}
		});
		addMenu(submenu, "Set &all interval\tCtrl+A", SWT.MOD1 + 'A', new Runnable() {
			public void run() {
				MotionAnnotation ann = newAnnotation();
				if (ann == null) return;
				ann.startFrame = Configuration.BLEND_MARGIN;
				int mLength = motionViewer.getNavigator().getViewer().getMotionDataList()[0].motionList.size();
				ann.endFrame = mLength - 1 - Configuration.BLEND_MARGIN;
			}
		});
		addMenu(submenu, "&Delete label\tCtrl+D", SWT.MOD1 + 'D', new Runnable() {
			public void run() {
				MotionAnnotation annotation = timeline.getSelectedAnnotation();
				if (annotation == null) return;
				annotationList.remove(annotation);
				timeline.selectAnnotation(null);
				timeline.callRedraw();
				
			}
		});
		addMenu(submenu, "&Make Opposite Label\tCtrl+O", SWT.MOD1 + 'O', new Runnable() {
			public void run() {
				MotionAnnotation ann = timeline.getSelectedAnnotation();
				if (ann == null || ann.oppositePerson == 0 || ann.oppositePerson == ann.person) return;
				timeline.setSelectedPerson(ann.oppositePerson);
				newAnnotation();
				MotionAnnotation newAnn = timeline.getSelectedAnnotation();
				newAnn.type = ann.type;
				newAnn.startFrame = ann.startFrame;
				newAnn.endFrame = ann.endFrame;
				onAnnPropertyChange();
			}
		});
		addMenu(submenu, "&Update Annotation List\tCtrl+U", SWT.MOD1 + 'U', new Runnable() {
			public void run() {
				updateAnnotationTable();
			}
		});
		addMenu(submenu, "Save current annotation\tCtrl+W", SWT.MOD1 + 'W', new Runnable() {
			public void run() {
				saveCurrentAnnotation();
			}
		});
		addMenu(submenu, "E&xit witout save\tCtrl+X", SWT.MOD1 + 'X', new Runnable() {
			public void run() {
				saveFile = false;
				shell.close();
			}
		});
		
		
		MenuItem macroItem = new MenuItem (bar, SWT.CASCADE);
		macroItem.setText ("M&acro");
		Menu macroSubMenu = new Menu (shell, SWT.DROP_DOWN);
		macroItem.setMenu (macroSubMenu);
		
//		propertyPanel.setSampleValues("type", "punck", "kick", "push", "slap", "others");
//		propertyPanel.setSampleValues("subtype", "", "side", "in-out", "down-up");
//		propertyPanel.setSampleValues("beforeActiveState", "standing", "bent", "lying");
//		propertyPanel.setSampleValues("afterActiveState", "standing", "bent", "lying");
//		propertyPanel.setSampleValues("beforePassiveState", "standing", "bent", "lying");
//		propertyPanel.setSampleValues("afterPassiveState", "standing", "bent", "lying");
//		propertyPanel.setSampleValues("interactionType", "", "dodge", "threat");
//		propertyPanel.setSampleValues("interactionPassivePart", "", "head", "stomach", "chest", "leg", "foot", "arm");
		addMenu(macroSubMenu, "Set All Standing\tCtrl+1", SWT.MOD1 + '1', new Runnable() {
			public void run() {
				propertyPanel.setFieldValue("beforeActiveState", "standing");
				propertyPanel.setFieldValue("afterActiveState", "standing");
				propertyPanel.setFieldValue("beforePassiveState", "standing");
				propertyPanel.setFieldValue("afterPassiveState", "standing");
			}
		});
		addMenu(macroSubMenu, "Set Side Punch and Head\tCtrl+2", SWT.MOD1 + '2', new Runnable() {
			public void run() {
				propertyPanel.setFieldValue("subtype", "side");
				propertyPanel.setFieldValue("interactionPassivePart", "head");
			}
		});
	}
	
	protected void updateFileSelection(){
		saveCurrentAnnotation();
		
		int idx = fileList.getSelectionIndex();
		if (idx < 0) return;

		
		
		MultiFileMotionGetter getter = ((MultiFileMotionGetter)motionGetterList.get(idx));
		currentFileList = getter.getFiles().fileList;
		currentAnnotationFile = new File(outputFolder + "\\" + MultiCharacterFolder.getTakeName(currentFileList.get(0).getName()) + ".lab");
		
		if (currentAnnotationFile.exists()){
			String defaultType = MotionAnnotation.defaultType;
			annotationList = ObjectSerializer.load(MotionAnnotation.class, currentAnnotationFile);
			MotionAnnotation.defaultType = defaultType;
		} else {
			annotationList = new ArrayList<MotionAnnotation>();
		}
		
		MotionData[] motionList = motionGetterList.get(idx).getMotionDataList();
		for (MotionData motionData : motionList){
			FootContactDetection.checkFootContact(motionData);
		}
		motionViewer.getNavigator().stopAnimation();
		motionViewer.setMotionData(motionList);
		motionViewer.getNavigator().startAnimation();
		
		timeline.setMotionInfo(motionList.length, motionList[0].motionList.size());
		timeline.setAnnotationList(annotationList);
	}
	
	private int getCurrentFrame(){
		return motionViewer.getNavigator().getMotionIndex();
	}
	
	private MotionAnnotation newAnnotation(){
		int selectedPerson = timeline.getSelectedPerson();
		if (selectedPerson < 0) return null;
		MotionAnnotation annotation = new MotionAnnotation();
		annotation.person = selectedPerson;
		annotation.endFrame = annotation.startFrame = getCurrentFrame();
		annotation.file = currentFileList.get(annotation.person - 1).getName();
		annotationList.add(annotation);
		timeline.selectAnnotation(annotation);
		propertyPanel.setFieldFocus("type");
		return annotation;
	}
	
	private void onKeyDown(KeyEvent e){
	}
	
	protected void saveCurrentAnnotation(){
		if (!saveFile) return;
		if (currentAnnotationFile != null){
			if (annotationList.size() == 0 && !currentAnnotationFile.exists()) return;
			ObjectSerializer.save(MotionAnnotation.class, annotationList, currentAnnotationFile);
		}
	}
	
	@Override
	public void open(int width, int height){
		try {
			super.open(width, height);
		} catch (Throwable e) {
			saveCurrentAnnotation();
			e.printStackTrace();
		}
	}
	
	private void setSampleValues(){
		propertyPanel.setSampleValues("type", BALL_CONTACTS[0], BALL_CONTACTS[1], "walk", "punch", "kick", "push", "slap", "pointing", "grab", "others");
//		propertyPanel.setSampleValues("type", "walk", "punch", "kick", "push", "slap", "pointing", "grab", "others");
		propertyPanel.setSampleValues("subtype", "", "side", "in-out", "down-up");
		propertyPanel.setSampleValues("beforeActiveState", "standing", "bent", "lying");
		propertyPanel.setSampleValues("afterActiveState", "standing", "bent", "lying");
		propertyPanel.setSampleValues("beforePassiveState", "standing", "bent", "lying");
		propertyPanel.setSampleValues("afterPassiveState", "standing", "bent", "lying");
		propertyPanel.setSampleValues("interactionType", "", "dodge", "threat");
		
		String[] activeParts = FilterUtil.split("|head|stomach|chest|leg|foot|arm|shoulder");
		String[] passiveParts = FilterUtil.split("|head|stomach|chest|leg|foot|arm|shoulder");
		propertyPanel.setSampleValues("interactionActivePart", activeParts);
		propertyPanel.setSampleValues("interactionPassivePart", passiveParts);
	}
	
	public static void open(String motionFolder, String annFolder){
		MotionAnnotationHelper helper = new MotionAnnotationHelper(motionFolder, annFolder);
		helper.open();
	}
}
