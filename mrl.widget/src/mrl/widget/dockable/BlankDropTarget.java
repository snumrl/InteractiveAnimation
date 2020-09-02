package mrl.widget.dockable;

import org.eclipse.swt.graphics.Cursor;
import org.eclipse.swt.graphics.Rectangle;

/**
 * �־��� �������� Track Bounds�� �������ְ� ���� drop ������ ���� �ʴ� DropTarget
 * 
 * @author whcjs
 */
public class BlankDropTarget implements IDropTarget{
	
	private Rectangle snapRectangle;
	
	public BlankDropTarget(Rectangle snapRectangle) {
		this.snapRectangle = snapRectangle;
	}

	public void drop() {
	}

	public Cursor getCursor() {
		return null;
	}

	public Rectangle getSnapRectangle() {
		return snapRectangle;
	}

}
