package mrl.util.heap;

public class MinHeap<E extends HeapNode> {

	E[] nodeList;
	int maxIndex = 1;
	
	public MinHeap(E[] nodeList){
		this.nodeList = nodeList;
	}
	
	public int getCurrentSize(){
		return maxIndex - 1;
	}
	
	/**
	 * ���� heap�� node�� �� value�� ���� ���� node�� ��ȯ�ϴ� �Լ�
	 * @return
	 */
	public E getMinimum(){
		while (true){
			if (maxIndex <= 1) return null;
			// �⺻������ 1��° index�� �ִ� node�� ���� ���� value�� ���� node������,
			// �ӵ����� ������ Intrinsic dimensionality ������ Ȯ�� ���� �ʾ��� �� �ֱ� ������,
			// Ȯ���� �ȵ� ��� Ȯ�� �ϰ� ������ �������� ���ϸ� value�� infinity ������ �ΰ�
			// �ٽ� ���� value ���� ���� node�� ���� �̾Ƽ� Ȯ���ϴ� ������ �ݺ��Ѵ�.
			E node = nodeList[1];
			// �ش� node�� ���� intrinsic dimensionality ������ Ȯ���ϰ�
			// �ùٸ� ��쿡�� node�� ��ȯ�ϰ� �׷��� ������
			// �ش� node�� value�� �ٲ� ���� heap�� �籸���Ѵ�.
			if (node.validate()){
				return node;
			} else {
//					shiftDown(1);
				removeNode(node);
				node.onRemoveByValidation();
			}
		}
	}
	
	/**
	 * heap�� ���ο� node�� �߰��ϴ� �Լ�
	 * @param node
	 */
	public void addNode(E node){
		// �ϴ� �� �κп� node�� �߰���Ų��.
		nodeList[maxIndex] = node;
		node.index = maxIndex;
		maxIndex++;
		
		// ���� �߰��� ��ġ���� �θ� node�� recursive �ϰ� Ȯ���ϸ鼭
		// �θ��� value�� �� ũ�ٸ� �ڽŰ� �θ��� ��ġ�� �ٲ۴�.
		shiftUp(maxIndex - 1);
	}
	
	/**
	 * heap���� node�� �����ϴ� �Լ�
	 * @param node
	 */
	public void removeNode(E node){
//		if (node.index == -1) throw new RuntimeException();
		if (node.index == -1) return;
		
		if (node.index == maxIndex - 1){
			// �������� ��ġ�� node��� �׳� ���� maxIndex�� 1 ���̸� �ȴ�.
			maxIndex--;
			nodeList[maxIndex] = null;
			node.index = -1;
		} else {
			// �ٸ� ��ġ�� �ִ� node��� �� ������ ��ġ�� node�� ��ġ�� �ٲٰ� 
			// ���������� maxIndex�� 1 �ٿ��� �ش� ��带 ����.				 
			E movedNode = nodeList[maxIndex - 1];
			nodeList[node.index] = movedNode;
			movedNode.index = node.index;
			nodeList[maxIndex - 1] = null;
			maxIndex--;
							
			
			// ���� ��ġ�� �ٲ� ��忡 ���ؼ� �ڽİ� �θ��� ���� ���� ������ ��ġ�� �����Ѵ�.
			shiftDown(node.index);
			if (nodeList[node.index] == movedNode){
				shiftUp(node.index);
			}
			
			// ���ŵ� ����� index�� -1�� �ξ� ������ ������� ǥ���Ѵ�.
			node.index = -1;
		}
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
		E temp = nodeList[i1];
		nodeList[i1] = nodeList[i2];
		nodeList[i2] = temp;
		nodeList[i1].index = i1;
		nodeList[i2].index = i2;
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
}
