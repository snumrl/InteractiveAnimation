package mrl.widget.dockable;

import org.eclipse.swt.custom.CTabItem;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.widgets.Control;

/**
 * ����ڰ� DockableTabFolder ���� �巡�׸� �Ͽ�����,
 * �� ��ġ�� TabFolder�� Tab����� ��Ȯ�� � Item ���������� ����Ͽ�
 * �� ��ġ�� �µ��� �����ִ� DropTarget�� �����ϴ� Ŭ����
 * 
 * @author whcjs
 */
public class TabFolderDragOverListener implements IDragOverListener{
	
	private DockableTabFolder dockableTabfolder;

	public TabFolderDragOverListener(DockableTabFolder dockableTabfolder) {
		this.dockableTabfolder = dockableTabfolder;
	}
	
	private boolean isAssignable(Object draggedObject){
		return (draggedObject instanceof CTabItem
				 || draggedObject instanceof DockableTabFolder);
	}
	
	public IDropTarget drag(Control currentControl, Object draggedObject,
							Point position, Rectangle dragRectangle) {
		
		// isAssignable
		if (!isAssignable(draggedObject)) return null;
		
		Rectangle folderBounds = DragUtil.toDisplayBounds(dockableTabfolder);
		
		if (draggedObject == dockableTabfolder){
			return new BlankDropTarget(folderBounds);
		}
		
		Rectangle snapRectangle = null;
		Rectangle barBounds = new Rectangle(folderBounds.x, folderBounds.y, 
				folderBounds.width, dockableTabfolder.getTabHeight());
		if (barBounds.contains(position)){
			int targetIndex = -1;
			if (draggedObject instanceof CTabItem){
				for (CTabItem item : dockableTabfolder.getItems()){
					Rectangle itemBounds = DragUtil.toDisplayBounds(item.getBounds(), dockableTabfolder);
					if (itemBounds.contains(position)){
						snapRectangle = itemBounds;
						targetIndex = dockableTabfolder.indexOf(item);
						break;
					}
				}
				CTabItem draggedItem = (CTabItem)draggedObject;
				if (draggedItem.getParent() == dockableTabfolder){
					int index = dockableTabfolder.indexOf(draggedItem);
					if (targetIndex > index){
						targetIndex++;
					}
				}
			}
			if (snapRectangle == null){
				snapRectangle = barBounds;
			}
			return new TabFolderTabDropTarget(draggedObject, dockableTabfolder, targetIndex, snapRectangle);
		}
		
		return null;
	}
	
}
