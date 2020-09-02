package mrl.widget.dockable;

import mrl.widget.dockable.SashFormContainer.DockingPosition;

import org.eclipse.swt.custom.CTabItem;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;

/**
 * ����ڰ� �巡�׸� �Ͽ� ������ SashFormContainer���� �����������
 * �ش� SashFormContainer�� ��� ��ġ�� �������� ����Ͽ� �׿� �´� DropTarget�� �����ϴ� Ŭ����
 * 
 * @author whcjs
 */
public class SashFormDragOverListener implements IDragOverListener{
	
	private SashFormContainer targetContainer;
	private int parentSashFormCount;
	
	public SashFormDragOverListener(SashFormContainer targetContainer) {
		this.targetContainer = targetContainer;
		
		parentSashFormCount = 0;
		Composite parent = targetContainer.getParent();
		while (parent != null){
			if (parent instanceof SashFormContainer){
				parentSashFormCount++;
			}
			parent = parent.getParent();
		}
	}
	
	private boolean isAssignable(Object draggedObject){
		return (draggedObject instanceof CTabItem
				 || draggedObject instanceof DockableTabFolder);
	}

	public IDropTarget drag(Control currentControl, Object draggedObject,
			Point position, Rectangle dragRectangle) {
		
		// isAssignable
		if (!isAssignable(draggedObject)) return null;
		
		Rectangle folderBounds = DragUtil.toDisplayBounds(targetContainer);
		int topDistance = position.y - folderBounds.y;
		int bottomDistance = folderBounds.y + folderBounds.height - position.y;
		int leftDistance = position.x - folderBounds.x;
		int rightDistance = folderBounds.x + folderBounds.width - position.x;
		
		int horizontalMin = Math.min(leftDistance, rightDistance);
		int verticalMin = Math.min(topDistance, bottomDistance);
		int minimum = Math.min(horizontalMin, verticalMin);
		
		if (minimum > 120){
			// �߾����� �巡�� �� ��� �׳� �÷����� �ǵ��� �Ѵ�.
			return null;
		} else if (parentSashFormCount > 0 && minimum < parentSashFormCount * 10){
			// �θ� SashForm�� �ְ� ���� �������� �巡�� �� ���, �θ𿡼� ó���ϵ��� null�� ��ȯ�Ѵ�.
			return null;
		}
		
		DockingPosition dockingPosition;
		if (verticalMin < horizontalMin){
			dockingPosition = (topDistance < bottomDistance) ? DockingPosition.Top : DockingPosition.Bottom;
		} else {
			dockingPosition = (leftDistance < rightDistance) ? DockingPosition.Left : DockingPosition.Right;
		}
		
		return new DockingDropTarget(draggedObject, dockingPosition, 
										targetContainer, folderBounds);
	}
	
}
