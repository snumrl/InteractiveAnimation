package mrl.motion.neural.run;

import jep.Jep;
import jep.JepConfig;
import jep.JepException;
import jep.NDArray;

public class RNNPython {

	private Jep jep;
	
	public RNNModel model;
	public String name;

	public RNNPython(String name) {
		this(name, false);
	}
	public RNNPython(String name, boolean prefix) {
		try {
			this.name = name;
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
			
			model = loadModel(name, prefix);
		} catch (JepException e) {
			throw new RuntimeException(e);
		}
	}
	
	private void eval(String code) throws JepException{
		for (String c : code.split("\n")){
			jep.eval(c);
		}
	}
	
	public RNNModel loadModel(String name, boolean prefix){
		try {
			return new RNNModel(name, prefix);
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

		public RNNModel(String name, boolean prefix) throws JepException {
			this.name = name;
			
			for (String v : variables){
				eval(name + "_" + v + " = None");
			}
			
			jep.set("trainedFolder", "neuralData\\" + name + "\\train\\ckpt");
			eval("c = cf.get_config(\"" + name + "\")\n" +
					"c.load_normal_data(\"neuralData\\\\" + name + "\")\n" +
					"c.INPUT_KEEP_PROB = 1\n"+
					"c.LAYER_KEEP_PROB = 1\n"+
					name + "_m = c.model(1, 1)\n"
					);
			restore(prefix);
			
			
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
		
		public double[] predict(double[] target) {
			try {
				preProcess();
				jep.set("b_z", target);
				jep.eval("feed_dict = { m.x:[[b_z]], m.prev_y:current_y }");
				if (isFirst){
					isFirst = false;
				} else {
					jep.eval("feed_dict[m.initial_state] = state");
				}
				jep.eval("output, state, current_y = sess.run([m.generated, m.final_state, m.final_y], feed_dict)");
				postProcess();
				
				double[] generated = getValues("output");
				return generated;
			} catch (JepException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		}
		
		public void setStartMotion(double[] data){
			try {
				isFirst = true;
				jep.set("b_z", data);
				jep.eval(name + "_current_y = [b_z]");
			} catch (JepException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
		}
		
		public void setStartMotionOnly(double[] data){
			try {
				jep.set("b_z", data);
				jep.eval(name + "_current_y = [b_z]");
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
//						"for v in t_variables:\n\tprint(v.get_shape())\n",
//						"sys.stdout.flush()",
						"saver = tf.train.Saver(t_variables)",
						"saver.restore(sess, trainedFolder)",
				};
			}
			for (String c : codes){
				jep.eval(c);
			}
		}
	}
	
	private double[] getValues(String name) throws JepException{
		Object bbb = jep.getValue(name);
		NDArray<?> array = (NDArray<?>) bbb;
		float[] values = ((float[]) array.getData());
		double[] ret = new double[values.length];
		for (int i = 0; i < ret.length; i++) {
			ret[i] = values[i];
		}
		return ret;
	}

	
}
