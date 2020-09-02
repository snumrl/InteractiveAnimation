package mrl.widget.dockable;

import mrl.widget.dockable.SashFormContainer.DockingPosition;

import org.eclipse.swt.custom.CTabItem;
import org.eclipse.swt.graphics.Cursor;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.widgets.Control;

/**
 * SashFormContainer �� Ư�� ��ġ�� �巡�׵� Item�� ���̴� ������ �ϴ� DropItem
 * 
 * CTabItem�̳� DockableTabFolder�� �޾Ƶ鿩��
 * targetContainer�� ������ ��ġ�� �ٿ��ش�.
 * 
 * @author whcjs
 */
public class DockingDropTarget implements IDropTarget{
	
	private Object draggedObject;
	private DockingPosition dockingPosition;
	private SashFormContainer targetContainer;
	
	private Rectangle snapRectangle;

	public DockingDropTarget(Object draggedObject, DockingPosition dockingPosition, SashFormContainer targetContainer, Rectangle targetControlBounds) {
		this.draggedObject = draggedObject;
		this.dockingPosition = dockingPosition;
		this.targetContainer = targetContainer;
		
		switch (dockingPosition) {
		case Top:
			snapRectangle = new Rectangle(targetControlBounds.x, targetControlBounds.y, 
							targetControlBounds.width, targetControlBounds.height/3);
			break;
		case Bottom:
			snapRectangle = new Rectangle(targetControlBounds.x, 
							targetControlBounds.y + targetControlBounds.height*2/3, 
							targetControlBounds.width, targetControlBounds.height/3);
			break;
		case Left:
			snapRectangle = new Rectangle(targetControlBounds.x, targetControlBounds.y, 
							targetControlBounds.width/3, targetControlBounds.height);
			break;
		case Right:
			snapRectangle = new Rectangle(targetControlBounds.x + targetControlBounds.width*2/3, 
							targetControlBounds.y, 
							targetControlBounds.width/3, targetControlBounds.height);
			break;
		}
	}

	public void drop() {
		if (draggedObject instanceof CTabItem){
			CTabItem item = (CTabItem) draggedObject;
			DockableTabFolder newFolder = new DockableTabFolder(targetContainer, item.getParent().getStyle());
			targetContainer.dropNewControl(newFolder, dockingPosition, snapRectangle);
			TabFolderTabDropTarget.dropTabItem(item, newFolder, -1);
		} else if (draggedObject instanceof Control){
			targetContainer.dropNewControl((Control)draggedObject, dockingPosition, snapRectangle);
		}
	}

	public Cursor getCursor() {
		return null;
	}

	public Rectangle getSnapRectangle() {
		return snapRectangle;
	}

}
