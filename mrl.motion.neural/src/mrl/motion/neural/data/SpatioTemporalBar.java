package mrl.motion.neural.data;

import mrl.motion.neural.data.ParameterBar.AngleBar;
import mrl.motion.neural.data.ParameterBar.LengthBar;

public class SpatioTemporalBar {

	public ParameterBar posAngleBar;
	public ParameterBar posLengthBar;
	public ParameterBar dirAngleBar;
	public ParameterBar timeBar;
	
	public SpatioTemporalBar(int barSizeScale, double minTime, double maxTime, double maxLength){
		posAngleBar = new AngleBar(barSizeScale*2, 2);
		posLengthBar = new LengthBar(0, maxLength, barSizeScale*3, 2);
		dirAngleBar = new AngleBar(barSizeScale*2, 2);
		timeBar = new LengthBar(-minTime, maxTime, barSizeScale*2, 2);
	}
}
