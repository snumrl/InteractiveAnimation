package mrl.motion.neural.run;

import mrl.motion.neural.data.Normalizer;
import jep.Jep;
import jep.JepConfig;
import jep.JepException;
import jep.NDArray;

public class MultiPersonRNNPython {

	private Jep jep;
	
	public RNNModel model;
	public String name;
	public int persons;
	public Normalizer normal;

	public MultiPersonRNNPython(String name, int persons) {
		normal = new Normalizer(name);
		try {
			this.name = name;
			this.persons = persons;
			JepConfig config = new JepConfig();
			config.addIncludePaths("..\\mrl.python.neural");
			jep = new Jep(config);
			jep.eval("import sys");
			jep.eval("sys.argv = ['']");
			
			
			String code = 
					"import rnn.RNNModel as rm\n"+
					"import rnn.Configurations as cf\n"+
					"import tensorflow as tf\n"+
					"sess = tf.Session()\n";
			for (String c : code.split("\n")){
				jep.eval(c);
			}
			
			model = loadModel(name);
		} catch (JepException e) {
			throw new RuntimeException(e);
		}
	}
	
	private void eval(String code) throws JepException{
		for (String c : code.split("\n")){
			jep.eval(c);
		}
	}
	
	public RNNModel loadModel(String name){
		try {
			return new RNNModel(name);
		} catch (JepException e) {
			throw new RuntimeException(e);
		}
	}
	
	public class RNNModel{
		public String name;
		private boolean isFirst = true;
		
		private String[] variables = {
				"m",
				"current_y",
				"state",
			};

		public RNNModel(String name) throws JepException {
			this.name = name;
			
			for (String v : variables){
				eval(name + "_" + v + " = None");
			}
			
			jep.set("trainedFolder", "neuralData\\" + name + "\\train\\ckpt");
			eval("c = cf.get_config(\"" + name + "\")\n" +
					"c.load_normal_data(\"neuralData\\\\" + name + "\")\n" +
					"c.INPUT_KEEP_PROB = 1\n"+
					"c.LAYER_KEEP_PROB = 1\n"+
					name + "_m = c.model(" + persons + ", 1)\n"
					);
			restore(false);
			
			
		}
		
		private void preProcess() throws JepException{
			for (String v : variables){
				eval(v + " = " + name + "_" + v);
			}
		}
		private void postProcess() throws JepException{
			for (String v : variables){
				if (v.equals("m")) continue;
				eval(name + "_" + v + " = " + v);
			}
		}
		
		public double[][] predict(double[][] target) {
			try {
				preProcess();
				
				double[][][] edited = new double[target.length][1][target[0].length];
				for (int i = 0; i < edited.length; i++) {
					edited[i][0] = target[i];
				}
				
				jep.set("b_z", edited);
				jep.eval("feed_dict = { m.x:b_z, m.prev_y:current_y }");
				if (isFirst){
					isFirst = false;
				} else {
					jep.eval("feed_dict[m.initial_state] = state");
				}
				jep.eval("output, state, current_y = sess.run([m.generated, m.final_state, m.final_y], feed_dict)");
				postProcess();
				
				double[][] generated = getValues("output");
				return generated;
			} catch (JepException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		}
		
		public void setStartMotion(double[][] data){
			try {
				isFirst = true;
				jep.set("b_z", data);
				jep.eval(name + "_current_y = b_z");
			} catch (JepException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		}
		
		private void restore(boolean prefix) throws JepException{
			String[] codes = null;
			if (prefix){ 
				codes = new String[]{
						"prefix = c.label",
						"t_variables = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=prefix)",
						"p_len = len(prefix) + 1",
						"v_dict = {}",
						"for v in t_variables:\n\tv_dict[v.name[p_len:-2]] = v\n",
						"saver = tf.train.Saver(v_dict)",
						"saver.restore(sess, trainedFolder)",
				};
			} else {
				codes = new String[]{
//						"vars_in_checkpoint = tf.train.list_variables(trainedFolder)",
//						"for v in vars_in_checkpoint:\n\tprint(v)\n",
//						"sys.stdout.flush()",
						"prefix = c.label",
						"t_variables = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope=prefix)",
						"saver = tf.train.Saver(t_variables)",
						"saver.restore(sess, trainedFolder)",
				};
			}
			for (String c : codes){
				jep.eval(c);
			}
		}
	}
	
	private double[][] getValues(String name) throws JepException{
		Object bbb = jep.getValue(name);
		NDArray<?> array = (NDArray<?>) bbb;
		float[] values = ((float[]) array.getData());
		int[] dimensions = array.getDimensions();
		double[][] ret = new double[dimensions[0]][dimensions[2]];
		for (int i = 0; i < ret.length; i++) {
			for (int j = 0; j < ret[i].length; j++) {
				ret[i][j] = values[i*dimensions[2] + j];
			}
		}
		return ret;
	}

	
}
