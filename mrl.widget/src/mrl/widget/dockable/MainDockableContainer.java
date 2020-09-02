package mrl.widget.dockable;

import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CTabFolder;
import org.eclipse.swt.custom.CTabItem;
import org.eclipse.swt.custom.StackLayout;
import org.eclipse.swt.events.DisposeEvent;
import org.eclipse.swt.events.DisposeListener;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Shell;

/**
 * Floating/Docking�� �����ϴ� ��Ʈ Container�� �ش��ϴ� Ŭ����
 * 
 * �� Container ������ ���� Control�� �ȿ��� Floating/Docking�� ������ �ǰ� �ȴ�.
 * 
 * Floating���� ���� �� Shell�� ���������δ� �� Container�� ���ϰ� �Ǿ�,
 * �� Container�� ������ Floating���� �� �ִ� Shell�鵵 ������ �ȴ�.
 * 
 * �׹ۿ� ��� Control�� Tab�� �������� ���ܵ״ٰ� ���߿� �״�� �ٽ� ����� �ϴ� ��찡 ������,
 * UndisposableTabControl�� ����Ͽ� �����ϸ� �ǰ�,
 * �ش� Control���� �� Container�� ��� �ְ� �ȴ�.
 * (StackLayout�� �̿��Ͽ� ǥ�������δ� �ܼ��� SashFormContainer�� �߰�
 *  ������ �մ� Control���� ǥ�ð� ���� �ʴ´�. )
 * 
 * @author whcjs
 */
public class MainDockableContainer extends Composite{
	
	private SashFormContainer container;
	
	public MainDockableContainer(Composite parent, int style) {
		super(parent, SWT.NONE);
		
		StackLayout layout = new StackLayout();
		setLayout(layout);
		
		container = new SashFormContainer(this, style);
		layout.topControl = container;
		
		addDisposeListener(new DisposeListener() {
			public void widgetDisposed(DisposeEvent e) {
				for (Shell shell : FloatingDropTarget.getChildShells(MainDockableContainer.this)){
					shell.close();
				}
				setRedraw(false);
			}
		});
	}
	
	public SashFormContainer getContainer() {
		return container;
	}
	
	public DockableTabFolder createFloatingTabFolder(){
		return FloatingDropTarget.createFloatingTabFolder(this);
	}
	
	public boolean showTabItem(String name){
		CTabItem item = findTabItem(name);
		if (item == null) return false;
		item.getParent().setSelection(item);
		return true;
	}
	
	public CTabItem findTabItem(String name){
		CTabItem item = findTabItem(name, this);
		if (item != null) return item;
		
		for (Shell shell : FloatingDropTarget.getChildShells(MainDockableContainer.this)){
			item = findTabItem(name, shell);
			if (item != null) return item;
		}
		
		return null;
	}
	
	public void setChildShellVisible(boolean visible){
		for (Shell shell : FloatingDropTarget.getChildShells(MainDockableContainer.this)){
			shell.setVisible(visible);
		}
	}
	
	private static CTabItem findTabItem(String name, Composite root){
		for (Control child : root.getChildren()){
			if (child instanceof CTabFolder){
				CTabFolder folder = (CTabFolder)child;
				for (CTabItem item : folder.getItems()){
					if (item.getText().equals(name)){
						return item;
					}
				}
			} else if (child instanceof Composite){
				CTabItem item = findTabItem(name, (Composite)child);
				if (item != null){
					return item;
				}
			}
		}
		return null;
	}
	
	static MainDockableContainer getParentMainContainer(Control control){
		if (control == null) return null;
		if (control instanceof MainDockableContainer){
			return (MainDockableContainer)control;
		}
		return getParentMainContainer(control.getParent());
	}

}
