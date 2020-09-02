package mrl.motion.neural.basket.sketch;

import javax.vecmath.Point2d;

public class SketchNode {
	
	public static final int TEAM_ATTACK = 0;
	public static final int TEAM_DEFENCE = 1;
	
	public static final String TEAM_ATTACK_PREFIX = "A";
	public static final String TEAM_DEFENCE_PREFIX = "D";

	public Point2d position;
	public String label;
	public int team;
	public boolean isShoot = false;
	
}
