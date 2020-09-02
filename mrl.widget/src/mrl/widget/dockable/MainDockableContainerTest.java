package mrl.widget.dockable;

import mrl.widget.dockable.SashFormContainer.DockingPosition;

import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CTabFolder;
import org.eclipse.swt.custom.CTabItem;
import org.eclipse.swt.events.KeyAdapter;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;

public class MainDockableContainerTest {
	
	private static int itemCount = 0;
	
	public static void main(String[] args) {
		Display display = new Display ();
		final Shell shell = new Shell (display);
		shell.setSize(800, 600);
		shell.setLayout(new FillLayout());
		
		// Docking ���� ������ MainContainer�� �����մϴ�.
		// Main Container ������ TabFolder �� Tab Item�� 
		// �� Main Container �����θ� Docking�� �����մϴ�.
		final MainDockableContainer mainContainer = new MainDockableContainer(shell, SWT.NONE);
		// ������ ���� Control���� �� MainContainer�� Container�ȿ� �ֽ��ϴ�.
		SashFormContainer container = mainContainer.getContainer();
		
		// �߾ӿ� Text Control�� �ֽ��ϴ�. �θ�� ������ ���� SahsFormContainer�� �ݴϴ�.
		final Text myText = new Text(container, SWT.BORDER | SWT.MULTI);
		myText.setText("Main Text");
		myText.setBackground(display.getSystemColor(SWT.COLOR_YELLOW));
		// SashFormContainer�� �߰������� �� Text Control�� ������־�� �մϴ�.
		container.dropInitialControl(myText);
		
		// �ι�° Control���� DockableTabFolder�� �����մϴ�. 
		// ���������� SashFromContainer�� �θ�� �մϴ�.
		// ( DockableTablFolder�� ������ TabItem���� Docking�� �����մϴ�. )
		DockableTabFolder folder = new DockableTabFolder(container, SWT.BORDER);
		folder.setSimple(true);
		// �ι�° Control�� SashFromContainer�� ����ϸ鼭 ��� ��ġ��ų���� �����մϴ�.
		container.dropNewControl(folder, DockingPosition.Right, null);
		
		// SashFormContainer�� ��/��, Ȥ�� ��/�Ʒ� ������ �����մϴ�.
		container.setWeights(new int[]{ 7, 3 });
		
		// TabFolder�� TabItem���� �����մϴ�.
		// �Ϲ� CTabFolder�� �����ϰ� Item ���� �����ϸ� �˴ϴ�.
		CTabItem item = new CTabItem(folder, SWT.BORDER | SWT.CLOSE);
		item.setText("Restorable");
//		item.setImage(getIcon());
		// TabItem�� Control�� UndisposableTabControl�� �����ϸ�
		// �ش� Control�� TabItem�� ������ dispose���� �ʰ� ���� ������,
		// restoreControl�Լ��� ���� �ٽ� ��� �� �ֽ��ϴ�.
		final UndisposableTabControl restoreControl = new UndisposableTabControl(item, mainContainer);
		Text restoreText = new Text(restoreControl, SWT.BORDER);
		restoreText.setText("Restorable");
		item.setControl(restoreControl);
		
		// �׳� �Ϲ� ���� �������� �����մϴ�.
		createSampleItems(folder);
		
		
		// �׽�Ʈ�� Ű �׼��� ��ϴ�.
		myText.addKeyListener(new KeyAdapter() {
			private boolean childShellVisible = true;
			
			public void keyPressed(KeyEvent e) {
				if (e.character == 'r'){
					// UndisposableTabControl�� restoreControl �Լ���
					// ������ �ִ� Control�� �ٽ� TabItem�� �Բ� ��� �� �ֽ��ϴ�.
					restoreControl.restoreControl(DockingPosition.Right, 
							(SashFormContainer)myText.getParent(), new int[]{ 7, 3 });
				}
				if (e.character == 'h'){
					// Floating �� Shell���� ���������� MainContainer�� ������ ���� ������
					// MainContainer�� setChildShellVisible �Լ��� ����ų� �ٽ� ���̰� �� �� �ֽ��ϴ�.
					childShellVisible = !childShellVisible;
					mainContainer.setChildShellVisible(childShellVisible);
				}
				if (e.character == 'f'){
					// TabItem���� �̸� ���� �Ű� �ٴ� �� �ְ�, Floating�� �Ǿ� ���� ���� �ִµ�,
					// ���������� �� MainContainer�ȿ� ���ϱ� ������,
					// MainContainer�� findTabItem�Լ��� �ش� �̸��� TabItem�� ã�� �� �ֽ��ϴ�.
					// ( ���� �̸��� TabItem�� ������ ���� ��� �� �� �ƹ��ų��� ��ȯ�˴ϴ�. )
					System.out.println(mainContainer.findTabItem("Restorable") != null);
				}
			}
		});
		
		shell.open ();
		while (!shell.isDisposed ()) {
			if (!display.readAndDispatch ()) display.sleep ();
		}
		display.dispose ();
	}
	
	private static void createSampleItems(DockableTabFolder folder){
		createItem(folder);
		createItem(folder);
		createItem(folder);
		folder.setSelection(0);
	}
	
	private static CTabItem createItem(CTabFolder tabFolder){
		itemCount++;
		String name = "Item " + itemCount;
		CTabItem item = new CTabItem(tabFolder, SWT.BORDER | SWT.CLOSE);
		Text control = new Text(tabFolder, SWT.BORDER);
		control.setText(name);
		item.setControl(control);
		item.setText(name);
		item.setImage(getIcon());
		return item;
	}
	
	private static Image icon;
	private static Image getIcon(){
		if (icon == null){
			Display display = Display.getCurrent();
			Image infoImage = display.getSystemImage(SWT.ICON_INFORMATION);
			Rectangle size = infoImage.getBounds();
			icon = new Image(display, 16, 16);
			GC gc = new GC(icon);
			gc.setAntialias(SWT.ON);
			gc.setAdvanced(true);
			gc.drawImage(infoImage, 0, 0, size.width, size.height, 0, 0, 16, 16);
			gc.dispose();
		}
		return icon;
	}
}
