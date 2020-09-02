package mrl.widget.app;

import mrl.widget.app.MainApplication.WindowPosition;

import org.eclipse.swt.widgets.Composite;

public abstract class Module {

	private MainApplication app;
	protected boolean isInitialized;
	
	public Module(){
	}
	
	public MainApplication app(){
		return app;
	}

	protected void setApplication(MainApplication application) {
		this.app = application;
	}
	
	protected <E extends Module> E getModule(Class<E> c){
		return app.getModule(c);
	}
	
	protected Composite dummyParent(){
		return app.dummyParent();
	}
	
	protected <E extends Composite> E addWindow(E window,WindowPosition position){
		return app.addWindow(window, getName(), position);
	}
	
	/**
	 * addMenu("&Menu", "&Test\tCtrl+T", SWT.MOD1 + 'T', new Runnable() { 
	 * @param parentName
	 * @param text
	 * @param accelerator
	 * @param runnable
	 */
	protected void addMenu(String parentName, String text, int accelerator, final Runnable runnable){
		app.addMenu(parentName, text, accelerator, runnable);
	}
	
	public void showTabItem(){
		app.showTabItem(getName());
	}
	
	public void initialize(){
		if (isInitialized) return;
		isInitialized = true;
		initializeImpl();
	}
	
	protected boolean isInitialized() {
		return isInitialized;
	}

	protected boolean isInitializeOnStart(){
		return true;
	}
	
	public String getName(){
		return getClass().getSimpleName().replace("Module", "");
	}
	
	protected void createMenus(){
	}
	
	protected abstract void initializeImpl();
}
