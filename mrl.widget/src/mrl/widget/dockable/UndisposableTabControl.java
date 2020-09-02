package mrl.widget.dockable;

import java.util.ArrayList;

import mrl.widget.dockable.SashFormContainer.DockingPosition;

import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CTabItem;
import org.eclipse.swt.custom.SWTCustomInner;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.widgets.Composite;

/**
 * TabFolder�� TabControl���� ���� Control ��
 * �ش� Tab�� �������� Control�� dispose���� �ʰ� ���ܵ״ٰ�
 * ���߿� �״�� �ٽ� ����ִ� ����� �����ϱ� ���� Ŭ����
 * 
 * TabControl���� �� Composite�� ����� ������ ���� Control�� �ڽ����� ����� �ȴ�.
 * ������ Control�� �ٽ� ��ﶧ�� restoreControl �Լ��� �̿��ϸ� �ȴ�.
 * 
 * @author whcjs
 */
public class UndisposableTabControl extends Composite{
	
	private MainDockableContainer container;
	private String text;
	private Image image;
	private boolean isClosable;
	
	private ArrayList<UndisposableTabControlListener> listeners = new ArrayList<UndisposableTabControlListener>();

	public UndisposableTabControl(CTabItem item, 
						final MainDockableContainer container) {
		super(item.getParent(), SWT.NONE);
		
		this.container = container;
		
		text = item.getText();
		image = item.getImage();
		isClosable = SWTCustomInner.isCTabItemClosable(item);
		
		this.setLayout(new FillLayout());
	}
	
	/**
	 * Tab�� ������ Control�� dispose ���� �ʵ��� �ϱ� ����
	 * �� Control�� �θ� main container�� �����ϴ� �Լ�.
	 */
	void switchParent(){
		container.setRedraw(false);
		this.setLayoutData(null);
		this.setParent(container);
		container.setRedraw(true);
		notifyStatusChanged();
	}
	
	public void restoreControl(DockingPosition position, SashFormContainer container, int[] weights){
		if (getParent() != this.container){
			return;
		}
		
		DockableTabFolder tabFolder = container.getExistingTabFolder(position);
		
		if (tabFolder == null){
			tabFolder = new DockableTabFolder(container, SWT.BORDER);
			container.dropNewControl(tabFolder, position, null);
			if (weights != null){
				container.setWeights(weights);
			}
		}
		this.setLayoutData(null);
		this.setParent(tabFolder);
		
		int style = isClosable ? SWT.CLOSE : SWT.NONE;
		CTabItem item = new CTabItem(tabFolder, style);
		item.setText(text);
		item.setImage(image);
		item.setControl(this);
		tabFolder.setSelection(item);
		
		notifyStatusChanged();
	}
	
	public String getText() {
		return text;
	}

	public void setText(String text) {
		this.text = text;
	}

	public Image getImage() {
		return image;
	}

	public void setImage(Image image) {
		this.image = image;
	}

	public boolean isClosable() {
		return isClosable;
	}

	public void setClosable(boolean isClosable) {
		this.isClosable = isClosable;
	}

	public MainDockableContainer getContainer() {
		return container;
	}
	
	public boolean isHided(){
		if (isDisposed()) return true;
		return (getParent() == container);
	}
	
	public void addListener(UndisposableTabControlListener listener){
		listeners.add(listener);
	}
	
	public void removeListener(UndisposableTabControlListener listener){
		listeners.remove(listener);
	}
	
	private void notifyStatusChanged(){
		for (UndisposableTabControlListener listener : listeners){
			listener.onStatusChanged();
		}
	}
	
}
