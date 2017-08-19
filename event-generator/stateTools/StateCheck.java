package stateTools;

import soccerControllers.*;
import soccerPlayers.*;


// State status options

	
public final class StateCheck{	
	public static boolean sonarSuccess = false;
	public static boolean modIrSuccess = false;
	public static boolean unModIrSuccess = false;
	
	public static float irModRead = -1;
	public static float irUnModRead = -1;
	public static float sonarRead = -1;
	
	public static boolean inGoalRange = false;
	public static boolean ballInFront = false;
	public static boolean ballClose = false;
	public static boolean ballKickable = false;
	public static boolean robotMoving = false;
	public static boolean robotTurning = false;
	public static boolean bifStateGen;
	
	
	public static boolean BallInFront(float sonar, float irMod, float irUnMod){
		if(sonarRead < Globals.SONAR_CLOSE_READING){
			return true;
		}
		//************** IR Check -- Modulated*****************//
		if(!Float.isNaN((irMod))){
			if(irMod == 0){
				return true;
			}
		}
		
		//************** IR Check -- UnModulated***************//
		if(!Float.isNaN(irUnMod)){
			if(irUnMod == 0){
				return true;
			}
		}
		return false;
	}
	
	public static boolean BallClose(float sonar){
		if(sonar < Globals.SONAR_CLOSE_READING)
			return true;
		return false;
	}
	
	public static boolean BallKickable(float sonar){
		if(sonar < Globals.SONAR_CLOSE_READING)
			return true;
		return false;
	}
	
	
	
	// NOTE: Returning true => that the robot should be in this state
	public static boolean TurnToBallState(Kicker currMK){
		if(!ballInFront && !ballClose && !ballKickable)
			return true;
		else
			return false;
	}
	
	// Going to ball state
	public static boolean GotoBallState(Kicker currMK){
		// Check if the robot should remain in the goto ball state
		if(ballInFront && robotMoving && !ballClose)
			return true;
		else
			return false;
	}
	
	// Turning to goal state
	public static boolean TurnToGoalState(Kicker currMK){
		// Ball should remain in front of the robot while turning
		if(ballInFront && ballClose && robotTurning)
			return true;
		else
			return false;
	}
	
	
	public static boolean DribbleBallState(Kicker currMK){
		// Ball should be near and in front robot while moving to goal
		if(!inGoalRange && ballInFront && ballClose)
			return true;
		else
			return false;
	}
	
	
	// Kick state
	public static boolean KickBallAtGoal(Kicker currMK){
		// Ball should be with the robot and in the goal range until kick
		if(inGoalRange && ballInFront && ballClose)
			return true;
		else
			return false;
	}
	
	
	// Get the state of the robot
	// NOTE: This may be the incorrect way of doing this -- depending on what is needed
	public static State GetState(ChangeEvent bifTriggered, Kicker currMK){
	
		//System.out.println("*PERFORMING STATE CHECK*");
		
		sonarRead = currMK.GetSensorController().GetLastSonar();
		// If any sensor reads as ball in front, then set the
		// appropriote flags
		if(bifTriggered != null)
			ballInFront = true;
		else
			ballInFront = false;
		
		// NOTE: this assumes that sonar is up to date
		ballClose = BallClose(sonarRead);
		ballKickable = BallKickable(sonarRead);
		
		
		
		inGoalRange = currMK.GetMotorController().InGoalRange();
		robotMoving = currMK.GetMotorController().RobotMoving();
		robotTurning = currMK.GetMotorController().RobotTurning();
		
		
		boolean inTurnToBallState = TurnToBallState(currMK);
		boolean inGotoBallState = GotoBallState(currMK);
		boolean inTurnToGoalState = TurnToGoalState(currMK);
		boolean inDribbleBallState = DribbleBallState(currMK);
		boolean inKickBallAtGoal = KickBallAtGoal(currMK);
		
		if(inTurnToBallState)
			return State.TURN_TO_BALL;
		else if(inGotoBallState)
			return State.GOTO_BALL;
		else if(inTurnToGoalState)
			return State.TURN_TO_GOAL;
		else if(inDribbleBallState)
			return State.DRIBBLE_TO_GOAL;
		else if(inKickBallAtGoal)
			return State.KICK_BALL_TO_GOAL;
		else
			return State.INIT;

	}
	
	// Print the current state of the robot
	public static void PrintState(State currState){
		switch(currState){
			case TURN_TO_BALL:
				System.out.println("In -TURN_TO_BALL- State");
				break;
			case TURN_TO_GOAL:
				System.out.println("In -TURN_TO_GOAL- State");
				break;
			case GOTO_BALL:
				System.out.println("In -GOTO_BALL- State");
				break;
			case DRIBBLE_TO_GOAL:
				System.out.println("In -DRIBBLE_TO_GOAL- State");
				break;
			case KICK_BALL_TO_GOAL:
				System.out.println("In -KICK_BALL_TO_GOAL- State");
				break;
			case INIT:
				
				break;
			default:
				System.out.println("UNDEFINED STATE");
				break;
		}
	}
}