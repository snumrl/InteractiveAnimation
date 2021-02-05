package mrl.graph;

import java.util.Collection;
import java.util.HashMap;

public class SparseMatrix<E> {

	private int size;
	private HashMap<Long, E> map = new HashMap<Long, E>();

	public SparseMatrix(int size) {
		this.size = size;
	}
	
	public boolean contains(int x, int y){
		return map.containsKey(key(x, y));
	}
	
	public E put(int x, int y, E e){
		return map.put(key(x, y), e);
	}
	
	public E get(int x, int y){
		return map.get(key(x, y));
	}
	
	private long key(int x, int y){
		return ((long)x)*size + y;
	}

	public Collection<E> values(){
		return map.values();
	}
}
