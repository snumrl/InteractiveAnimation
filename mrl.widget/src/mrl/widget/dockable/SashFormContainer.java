package mrl.widget.dockable;

import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CTabFolder;
import org.eclipse.swt.custom.CTabItem;
import org.eclipse.swt.custom.SashForm;
import org.eclipse.swt.events.DisposeEvent;
import org.eclipse.swt.events.DisposeListener;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Shell;

/**
 * SashForm���� ������ �ΰ��� ������ �ΰ��� Control�� ���� �� �ֵ��� �ϱ� ���� Composite
 * 
 * �ΰ��� ������ �ΰ��� SashFormContainer�� ����� �� Container�ȿ� ������ Control�� ��� ������, 
 * ��������� ����ؼ� Container�� ���� �� �ְ� �ϴ� ������ �Ǿ� �ִ�.
 * 
 * ���ԵǾ� �ִ� Control�� dispose�Ǹ� �׿� ���� �ڽ��� dispose �ǰų�
 * sash form�� ���ְ� �ٽ� �ϳ��� Control�� �㵵�� �����ϴ� ������ �����Ѵ�.
 * 
 * @author whcjs
 */
public class SashFormContainer extends Composite{
	
	private static final String DISPOSE_LISTENER_KEY = SashFormContainer.class.getName() + ".dispose";
	
	/**
	 * �ι�° Control�� ���� ������, ��� ��ġ�� ������������ �ش��ϴ� Ŭ����
	 * 
	 * @author whcjs
	 */
	public enum DockingPosition { 
		Left, Right, Top, Bottom; 
		
		public boolean isSplittedVertical(){
			return (this == Top || this == Bottom);
		}
		
		public boolean isAttachAfter(){
			return (this == Right || this == Bottom);
		}
	}
	
	private SashForm sashForm;
	
	private boolean isSetDisposed = false;
	
	public SashFormContainer(Composite parent, int style) {
		super(parent, style);
		
		addDisposeListener(new DisposeListener() {
			public void widgetDisposed(DisposeEvent e) {
				isSetDisposed = true;
			}
		});
		
		this.setLayout(new FillLayout());
		
		applyDisposeListener(this);
		DragUtil.setDragTarget(this, new SashFormDragOverListener(this));
	}
	
	public void setWeights(int[] weights){
		if (sashForm != null){
			sashForm.setWeights(weights);
		}
	}
	
	public CTabItem createNewDropItem(DockingPosition position, int[] defaultWeight, boolean isClosable){
		SashFormContainer container = this;
		
		DockableTabFolder tabFolder = container.getExistingTabFolder(position);
		
		boolean isTabFolderExisted = (tabFolder != null);
		if (!isTabFolderExisted){
			tabFolder = new DockableTabFolder(container, SWT.BORDER);
			container.dropNewControl(tabFolder, position, null);
			if (defaultWeight != null){
				container.setWeights(defaultWeight);
			}
		}
		
		CTabItem tabItem = new CTabItem(tabFolder, isClosable ? SWT.CLOSE : SWT.NONE);
		tabFolder.setSelection(tabItem);
		return tabItem;
	}
	
	/**
	 * ó���� �ϳ��� Control�� ����ϴ� �Լ�.
	 * 
	 * �Էµ� control�� �� Container�� ���� �� Control�� ���� �ְ�,
	 * �ٸ� Container�� ���� ������ Control �� ���� �ִ�.
	 * 
	 * @param control
	 */
	public void dropInitialControl(Control control){
		applyDisposeListener(control);
		
		if (control.getParent() != this){
			final Composite controlsParent = control.getParent();
			control.setParent(this);
			control.setLayoutData(null);
			if (controlsParent != this && controlsParent instanceof SashFormContainer){
				getDisplay().timerExec(1, new Runnable() {
					public void run() {
						if (controlsParent.isDisposed()) return;
						controlsParent.dispose();
					}
				});
			}
		}
	}

	/**
	 * �̹� �ϳ��� Control(Ȥ�� sashForm�� 2���� Control)�� �ִ� ��Ȳ����
	 * �Ǵٸ� Control�� ���� �ִ� �Լ�.
	 * 
	 * �Էµ� control�� �� SashFormContainer�� ��ȯ�Ѵ�.
	 * 
	 * @param control
	 * @param position
	 * @param controlBounds
	 */
	public SashFormContainer dropNewControl(Control control, DockingPosition position, Rectangle controlBounds){
		
		Control[] children = getChildren();
		if (sashForm != null && children.length > 2){
			throw new UnsupportedOperationException();
		}
		
		final Composite controlsParent = control.getParent();
		
		applyDisposeListener(control);
		
		SashForm oldSashForm = sashForm;
		int style = position.isSplittedVertical() ? SWT.VERTICAL : SWT.HORIZONTAL;
		sashForm = new SashForm(this, style);
		
		SashFormContainer container1 = new SashFormContainer(sashForm, getStyle());
		SashFormContainer container2 = new SashFormContainer(sashForm, getStyle());
		
		if (oldSashForm == null){
			Control originControl = (children[0] != control) ? children[0] : children[1];
			applyDisposeListener(originControl);
			
			if (position.isAttachAfter()){
				originControl.setParent(container1);
				control.setParent(container2);
			} else {
				originControl.setParent(container2);
				control.setParent(container1);
			}
			originControl.setLayoutData(null);
		} else {
			if (position.isAttachAfter()){
				container1.sashForm = oldSashForm;
				oldSashForm.setParent(container1);
				control.setParent(container2);
			} else {
				container2.sashForm = oldSashForm;
				oldSashForm.setParent(container2);
				control.setParent(container1);
			}
		}
		control.setLayoutData(null);
		
		Rectangle fullBounds = getBounds();
		if (controlBounds != null){
			int[] weights = new int[2];
			if (position.isSplittedVertical()){
				weights[0] = controlBounds.height;
				weights[1] = fullBounds.height - weights[0];
			} else {
				weights[0] = controlBounds.width;
				weights[1] = fullBounds.width - weights[0];
			}
			if (position.isAttachAfter()){
				int temp = weights[1];
				weights[1] = weights[0];
				weights[0] = temp;
			}
			sashForm.setWeights(weights);
		}
		
		this.layout();
		
		if (controlsParent != this && controlsParent instanceof SashFormContainer){
			getDisplay().timerExec(1, new Runnable() {
				public void run() {
					if (controlsParent.isDisposed()) return;
					controlsParent.dispose();
				}
			});
		}
		
		return position.isAttachAfter() ? container2 : container1;
	}
	
	public SashForm getSashForm() {
		return sashForm;
	}

	public boolean isSplitted(){
		return (sashForm != null);
	}
	
	public boolean isSplittedVertical(){
		return (sashForm.getStyle() & SWT.VERTICAL) != 0;
	}
	
	public SashFormContainer[] getSplittedContainers(){
		Control[] children = sashForm.getChildren();
		return new SashFormContainer[]{
			(SashFormContainer)children[0], (SashFormContainer)children[1]
		};
	}
	
	public DockableTabFolder getExistingTabFolder(DockingPosition position){
		if (sashForm != null) return null;
		
		if (getParent() instanceof SashForm){
			SashFormContainer parent = (SashFormContainer)getParent().getParent();
			DockableTabFolder target = getExistingTabFolder(parent, position);
			if (target != null){
				return target;
			} else if (parent.getParent() instanceof SashForm){
				target = getExistingTabFolder((SashFormContainer)parent.getParent().getParent(), position);
				if (target != null){
					return target;
				}
			}
		}
		
		return null;
	}
	
	private DockableTabFolder getExistingTabFolder(SashFormContainer parent, DockingPosition position){
		if (parent.isSplittedVertical() == position.isSplittedVertical()){
			int index = position.isAttachAfter() ? 1: 0;
			SashFormContainer target = parent.getSplittedContainers()[index];
			if (!target.isSplitted() && target.getChildren()[0] instanceof CTabFolder){
				return (DockableTabFolder)target.getChildren()[0];
			}
		}
		return null;
	}
	
	
	private void applyDisposeListener(Control control){
		if (control.getData(DISPOSE_LISTENER_KEY) == null){
			control.addDisposeListener(disposeListener);
		}
	}
	
	private static final ControlDisposeListener disposeListener = new ControlDisposeListener();
	
	private static class ControlDisposeListener implements DisposeListener{
		
		public void widgetDisposed(DisposeEvent e) {
			Control control = (Control)e.widget;
			
			checkFocusControl(control);
			
			final Composite parent = control.getParent();
			if (parent == null) return;
			
			if (parent instanceof Shell){
				// �θ� Shell�� ��� �ݾ��ش�.
				parent.getDisplay().timerExec(1, new Runnable() {
					public void run() {
						if (parent.isDisposed()) return;
						((Shell)parent).close();
					}
				});
			} else if (parent instanceof SashFormContainer){
				if (((SashFormContainer)parent).isSetDisposed){
					return;
				}
				
				// �θ� SashFormContainer �� ��� �� ������Ƿ� dispose ���ش�.
				parent.getDisplay().timerExec(1, new Runnable() {
					public void run() {
						if (parent.isDisposed()) return;
						parent.dispose();
					}
				});
			} else if (parent instanceof SashForm &&
						parent.getParent() instanceof SashFormContainer){
				if (((SashFormContainer)parent.getParent()).isSetDisposed){
					return;
				}
				
				// �θ� sashForm�̰� �� ���� �θ� SashFormContainer�� ���
				// sashForm�� �ΰ��� Control�� �������� �ִٰ� �ϳ��� ���� ���Ƿ�
				// sashForm�� ���ְ� �ٽ� �ϳ� ���� Control�� �θ� SashFormContainer�� �ٲ��ش�.
				final SashFormContainer container = (SashFormContainer)parent.getParent();
				Control[] children = container.sashForm.getChildren();
				Control remainControl = (children[0] == control) ? children[1] : children[0];
				if (remainControl instanceof SashFormContainer){
					SashFormContainer remainContainer = (SashFormContainer)remainControl;
					if (!remainContainer.isSplitted() && remainContainer.getChildren().length == 1){
						remainControl = remainContainer.getChildren()[0];
						remainContainer.removeDisposeListener(this);
					}
				}
				remainControl.setParent(container);
				remainControl.setLayoutData(null);
				container.sashForm.dispose();
				container.sashForm = null;
				container.getDisplay().timerExec(1, new Runnable() {
					public void run() {
						if (!container.isDisposed()){
							container.layout();
						}
					}
				});
			}
		}
		
		/**
		 * ���� ��Ŀ���� ���� Control�� �����鼭 ������ ����� ��찡 �־�,
		 * �������� Control�� ��Ŀ���� ���ִٸ� Shell�� ��Ŀ���� ������ �Ѵ�.
		 * 
		 * @param control
		 */
		private void checkFocusControl(Control control){
			Control focusControl = control.getDisplay().getFocusControl();
			if (focusControl == null || focusControl instanceof Shell){
				return;
			}
			
			Composite parent = control.getParent();
			Composite currentParent = focusControl.getParent();
			
			while (currentParent != null){
				if (parent == currentParent){
					control.getShell().setFocus();
					return;
				}
				currentParent = currentParent.getParent();
			}
		}
	}
}
