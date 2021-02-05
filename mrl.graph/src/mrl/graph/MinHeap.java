package mrl.graph;

public class MinHeap{

	private HeapNode[] nodeList;
	private int[] indexMap;
	/**
	 * ���� heap�� ����ִ� node���� ���� + 1�� ��.
	 * �� node�� ������ �� index�� ��带 �ְ�, �� index�� 1 ������Ų��.
	 */
	private int maxIndex = 1;
	
	public MinHeap(int size) {
		nodeList = new HeapNode[size+1];
		indexMap = new int[size];
		for (int i = 0; i < indexMap.length; i++) {
			indexMap[i] = -1;
		}
	}
	
	public boolean isEmpty(){
		return maxIndex <= 1;
	}
	
	public boolean contains(int index){
		return indexMap[index] >= 0;
	}
	
	public int removeMinimum(){
		HeapNode node = nodeList[1];
		// �� ������ ��ġ�� node�� ��ġ�� �ٲٰ� 
		// maxIndex�� 1 �ٿ��� �ش� ��带 ����.				 
		HeapNode movedNode = nodeList[maxIndex - 1];
		nodeList[1] = movedNode;
		indexMap[movedNode.index] = 1;
		nodeList[maxIndex - 1] = null;
		maxIndex--;
		
		indexMap[node.index] = -1;
		
		// ���� ��ġ�� �ٲ� ��忡 ���ؼ� �ڽİ� �θ��� ���� ���� ������ ��ġ�� �����Ѵ�.
		shiftDown(1);
		return node.index;
	}
	
	public void changeValue(int index, double value){
		HeapNode node = nodeList[indexMap[index]];
		node.value = value;
		shiftUp(indexMap[index]);
	}
	
	
	/**
	 * heap�� ���ο� node�� �߰��ϴ� �Լ�
	 * @param node
	 */
	public void addNode(int index, double value){
		HeapNode node = new HeapNode(index, value);
		// �ϴ� �� �κп� node�� �߰���Ų��.
		nodeList[maxIndex] = node;
		if (indexMap[index] >= 0) throw new RuntimeException();
		indexMap[index] = maxIndex;
		maxIndex++;
		
		// ���� �߰��� ��ġ���� �θ� node�� recursive �ϰ� Ȯ���ϸ鼭
		// �θ��� value�� �� ũ�ٸ� �ڽŰ� �θ��� ��ġ�� �ٲ۴�.
		shiftUp(maxIndex - 1);
	}
	
	/**
	 * �־��� index ��ġ���� �ڽ� ����� ���� ���ؼ�
	 * �ڽ� ������ ���� �� �۴ٸ� ���� ��ġ�� ���� �ٲ۵�,
	 * �Ʒ��� ������ ��ġ���� ���� �۾��� �ٽ� �ݺ��Ͽ�
	 * heap�� �ùٸ��� ������ �� �ֵ��� �Ѵ�.
	 * 
	 * @param index
	 */
	void shiftDown(int index){
		int child = index*2;
		if (child >= maxIndex) return;
		if (child + 1 < maxIndex){
			if (nodeList[child].value > nodeList[child + 1].value){
				child++;
			}
		}
		if (nodeList[index].value > nodeList[child].value){
			swap(index, child);
			shiftDown(child);
		}
	}
	
	/**
	 * �� ����� ��ġ�� ���� �ٲ۴�.
	 * @param i1
	 * @param i2
	 */
	void swap(int i1, int i2){
		HeapNode temp = nodeList[i1];
		nodeList[i1] = nodeList[i2];
		nodeList[i2] = temp;
		indexMap[nodeList[i1].index] = i1;
		indexMap[nodeList[i2].index] = i2;
	}
	
	/**
	 * �־��� index ��ġ���� �θ� ���� ���� ���ؼ�
	 * �θ� ����� ���� �� ũ�ٸ� ���� ��ġ�� ���� �ٲ۵�,
	 * ���� �ö� ��ġ���� ���� �۾��� �ٽ� �ݺ��Ͽ�
	 * heap�� �ùٸ��� ������ �� �ֵ��� �Ѵ�.
	 * @param index
	 */
	void shiftUp(int index){
		int parent = index/2;
		if (parent == 0) return;
		if (nodeList[index].value < nodeList[parent].value){
			swap(index, parent);
			shiftUp(parent);
		}
	}

	private static class HeapNode{
		public int index;
		public double value;
		
		public HeapNode(int index, double value) {
			this.index = index;
			this.value = value;
		}
	}
}
