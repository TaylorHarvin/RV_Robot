package soccerControllers;

import lejos.robotics.navigation.Waypoint;

public class Globals {
	public final static Waypoint GOAL_LOCATION = new Waypoint(100,10);
	public final static double GOAL_RANGE_THRESHOLD = 40;
	public final static int IR_UNMOD = 0;
	public final static int IR_MOD = 1;
	public final static float SONAR_CLOSE_READING = (float) 0.19;
	public final static float SONAR_IN_ARM_READING = (float) 0.07;
	public final static float SONAR_ERROR_VAL = 0;
	public final static float BUFFER_ANGLE = 30;
	public final static float MAX_MOTOR_SPEED = 30;
	public final static int FIND_BALL_TRY_MAX = 20;
	public final static int TURN_FAIL_SAFE_LIMIT = 15;
}
