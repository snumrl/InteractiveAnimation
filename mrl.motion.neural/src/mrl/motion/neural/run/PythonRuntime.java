package mrl.motion.neural.run;

import java.util.Arrays;

import mrl.motion.neural.data.Normalizer;
import jep.Jep;
import jep.JepConfig;
import jep.JepException;
import jep.NDArray;

public class PythonRuntime {

	private Jep jep;
	private boolean isFirst = true;
	public Normalizer normal;

	public PythonRuntime(String name, int batchSize) {
		normal = new Normalizer(name);
		try {
			JepConfig config = new JepConfig();
			config.addIncludePaths("..\\mrl.python.neural");
			jep = new Jep(config);
			jep.eval("import sys");
			jep.eval("sys.argv = ['']");
			jep.set("trainedFolder", "neuralData\\" + name + "\\train\\ckpt");
//			jep.set("trainedFolder", "C:\\Program Files\\PuTTY\\train\\ckpt");
			
			String code = 
					"import rnn.RNNModel as rm\n"+
					"import rnn.Configurations as cf\n"+
					"import tensorflow as tf\n"+
					"c = cf.get_config(\"" + name + "\")\n" +
					"m = c.runtime_model(" + batchSize + ")\n"+
					"sess = tf.Session()\n"+
					"saver = tf.train.Saver()\n"+
					"saver.restore(sess, trainedFolder)\n";
			for (String c : code.split("\n")){
				if (c.length() == 0) continue;
				jep.eval(c);
			}
		} catch (JepException e) {
			throw new RuntimeException(e);
		}
	}
	
	private double[][] getValues(String name) throws JepException{
		Object bbb = jep.getValue(name);
		NDArray<?> array = (NDArray<?>) bbb;
		float[] values = ((float[]) array.getData());
		int[] dimensions = array.getDimensions();
		double[][] ret = new double[dimensions[0]][dimensions[1]];
		for (int i = 0; i < ret.length; i++) {
			for (int j = 0; j < ret[i].length; j++) {
				ret[i][j] = values[i*dimensions[1] + j];
			}
		}
		return ret;
	}

	public double[][] predict(double[][] target) {
		try {
			jep.set("b_z", target);
			jep.eval("feed_dict = { m.x:b_z, m.prev_y:current_y }");
			if (isFirst){
				isFirst = false;
			} else {
				jep.eval("feed_dict[m.initial_state] = state");
			}
			jep.eval("current_y, state = sess.run([m.generated, m.final_state], feed_dict)");
			double[][] generated = getValues("current_y");
			return generated;
		} catch (JepException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}
	
	public double[][] estimate(double[][] goal) {
		try {
			jep.set("b_z", goal);
			jep.eval("feed_dict = { m.g:b_z, m.prev_y:current_y }");
			if (!isFirst){
				jep.eval("feed_dict[m.initial_state] = state");
			}
			jep.eval("[estimated] = sess.run([m.estimated], feed_dict)");
			double[][] generated = getValues("estimated");
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
			jep.eval("current_y = b_z");
		} catch (JepException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}
}
