package mrl.motion.neural.run;

import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

public class ResidualValidationModule extends Module{

	@Override
	protected void initializeImpl() {
		MotionDataConverter.setAllJoints();
		MotionDataConverter.setNoBall();
		String label = null;
//		String label = "bs_d20e";
		Normalizer normal = new Normalizer(label);
		System.out.println("dsize :: " + normal.xMeanAndStd[0].length + " : " + normal.yMeanAndStd[0].length);
		
		printInputInfo(normal);
		printOutputInfo(normal);
		System.exit(0);
		
		int len = Math.min(1000, normal.yList.size());
		int offset = 0;
		RuntimeMotionGenerator g = new RuntimeMotionGenerator();
		
		double[] currentPose = normal.xList.get(offset);
		currentPose = normal.deNormalizeX(currentPose);
		currentPose = Utils.cut(currentPose, currentPose.length-normal.yMeanAndStd[0].length, currentPose.length-1);
		for (int i = 0; i < len; i++) {
			double[] residual = normal.yList.get(i);
			residual = normal.deNormalizeY(residual);
			
			int poseStart = MotionDataConverter.ROOT_OFFSET;
			double[] output = MathUtil.copy(residual);
			for (int j = poseStart; j < output.length; j++) {
				output[j] = currentPose[j] + output[j];
			}
			g.update(output);
			currentPose = output;
		}
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("motion", g.motionSequence);
	}
	
	private void printInputInfo(Normalizer normal){
		System.out.println("######################");
		Normalizer.printDataInfo(new Object[]{
				"control", 2,
				"FootContact",2,
				"Root rotation", 1,
				"Root translation", 2,
				"Root Height", 1,
				
				"Head_End", 3,
				"LeftHand", 3,
				"LeftFoot", 3,
				"LeftToe_End", 3,
				"RightHand", 3,
				"RightFoot", 3,
				"RightToe_End", 3,
				
				"LeftArm", 3,
				"RightArm", 3,
				
				"LeftForeArm", 3,
				"LeftLeg", 3,
				"RightForeArm", 3,
				"RightLeg", 3,
				
				// added
				"Spine", 3,
				"LeftHand_End", 3,
				"RightHand_End", 3,
				"Neck", 3,
				"LeftUpLeg", 3,
				"RightUpLeg", 3,
		}, normal.xMeanAndStd);
	}
	private void printOutputInfo(Normalizer normal){
		System.out.println("######################");
		Normalizer.printDataInfo(new Object[]{
				"FootContact",2,
				"Root rotation", 1,
				"Root translation", 2,
				"Root Height", 1,
				
				"Head_End", 3,
				"LeftHand", 3,
				"LeftFoot", 3,
				"LeftToe_End", 3,
				"RightHand", 3,
				"RightFoot", 3,
				"RightToe_End", 3,
				
				"LeftArm", 3,
				"RightArm", 3,
				
				"LeftForeArm", 3,
				"LeftLeg", 3,
				"RightForeArm", 3,
				"RightLeg", 3,
				
				// added
				"Spine", 3,
				"LeftHand_End", 3,
				"RightHand_End", 3,
				"Neck", 3,
				"LeftUpLeg", 3,
				"RightUpLeg", 3,
		}, normal.yMeanAndStd);
	}

	public static void main(String[] args) {
		MainApplication.run(new ResidualValidationModule());
	}
}
