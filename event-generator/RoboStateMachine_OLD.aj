/* Developer: Taylor Harvin
 * Date Last Changed: 8/14/2016
 * Purpose: Provide state properties of the robot
 *
 */

import fullSoccer.*;
import stateTools.*;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.RegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.robotics.navigation.Navigator;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.HiTechnicCompass;
import lejos.hardware.sensor.HiTechnicIRSeekerV2;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

aspect RoboStateMachine{
	private long lastStateCheck = System.currentTimeMillis();
	private final long PING_TIME_LIMIT = 10000;
	private boolean sonarSet = false;
	private boolean irModSet = false;
	private boolean irUnModSet = false;
	private boolean armOpen = false;
	private ChangeEvent bifTrigger = null;
	
	//*************PC Flags************************
	private boolean bifFlag = false;
	private boolean bcFlag = false;
	private boolean bKickable = false;
	//*********************************************
	
	
	public boolean readyForStateCheck(Kicker MK){
		// All sensor values are set -- allow state check
		// Otherwise -- look at the 
		if(sonarSet && irModSet && irUnModSet)
			return true;
		else if(sonarSet && MK.getSensorControl().getLastSonar() <= SoccerGlobals.BALL_SONAR_DIST_GRAB)
			return true;
		else if(irModSet && MK.getSensorControl().getLastModIR() == 0)
			return true;
		else if(irUnModSet && MK.getSensorControl().getLastUnModIR() == 0)
			return true;
		else
			return false;
	}
	
	public void resetStatePreCheck(){
		sonarSet = false;
		irModSet = false;
		irUnModSet = false;
		
		bifFlag = false;
		bcFlag = false;
		bKickable = false;
		bifTrigger = null;
	}
	
	// Generate the ball in front true event on-demand rather than through Kicker
	public void Kicker.generateBallInFrontState(){
		System.out.println("Generated BIF");
	}
	
	// Check the current ball in front state from the Kicker (without triggering any events)
	public boolean Kicker.checkBallInFront(Kicker currMK){
		return currMK.ballInFront(true);
	}
	
	
	// Generate the ball close true event on-demand rather than through Kicker
	public void Kicker.generateBallCloseState(){
		System.out.println("Generated Ball Close");
	}
	
	// Check the current ball close state from the Kicker (without triggering any events)
	public boolean Kicker.checkBallClose(Kicker currMK){
		return currMK.ballClose(true);
	}
	
	
	// Generate the current state on-demand here rather than directly from state check method
	public void Kicker.generateStateEvent(State currState, Kicker currMK){
		switch(currState){
			case TURN_TO_BALL:
				System.out.println("GEN TURN_TO_BALL");
				// NOTE: Ensure that wonder event is triggered before BIF
				StateCheck.TurnToBallState(currMK);
				break;
			case GOTO_BALL:
				System.out.println("GEN GOTO_BALL");
				StateCheck.GotoBallState(currMK);
				break;
			case TURN_TO_GOAL:
				System.out.println("GEN TURN_TO_GOAL");
				StateCheck.TurnToGoalState(currMK);
				break;
			case DRIBBLE_TO_GOAL:
				System.out.println("GEN DRIBBLE_TO_GOAL");
				StateCheck.DribbleBallState(currMK);
				break;
			case KICK_BALL_TO_GOAL:
				System.out.println("GEN KICK_BALL_TO_GOAL");
				StateCheck.KickBallAtGoal(currMK);
				break;
			default:
				System.out.println(currState);
				System.out.println("GEN NONE");
				break;
		}
		
	}
	
	
	
	
	
	//POINTCUT SECTION**************************************************************************
	// General pointcut to allow for access to Kicker object for other pointcuts (through cflowbelow)
	pointcut playPC(Kicker MK) : call(public void Kicker.play()) && target(MK);
	pointcut ballInFrontPC(Kicker MK) : call(public boolean Kicker.ballInFront(boolean)) && target(MK);
	pointcut ballClosePC(Kicker MK) : call(public boolean Kicker.ballClose(boolean)) && target(MK);
	pointcut ballKickablePC(Kicker MK) : call(public boolean Kicker.ballKickable(boolean)) && target(MK);
	//ADVICE SECTION*****************************************************************************
	
	//BIF -- Cflow Section******************************************************************************
	// IR -- Mod advice, handle change in IR MOD value (after new ping)
	pointcut irModChange_BIF(Kicker MK) : cflowbelow(ballInFrontPC(MK)) && set(float SensorControl.ballDirMod)&& within(SensorControl);
	// IR -- Mod advice, handle change in IR MOD value (after new ping)
	after(Kicker MK, float newIrMod) :irModChange_BIF(MK) && args(newIrMod){
		irModSet = true;
		
		
		if(newIrMod == 0){
			bifFlag = true;
			bifTrigger = ChangeEvent.IR_MOD;
		}
		
		if(/*bifFlag ||*/ readyForStateCheck(MK)){
			System.out.println("***IR MOD Changed BIF***");
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	
	
	// IR -- Un-Mod advice, handle change in IR UN-MOD value (after new ping)
	pointcut irUnModChange_BIF(Kicker MK) : cflowbelow(ballInFrontPC(MK)) && set(float SensorControl.ballDirUnMod)&& within(SensorControl);
	// IR -- Un-Mod advice, handle change in IR UN-MOD value (after new ping)
	after(Kicker MK, float newIrUnMod):irUnModChange_BIF(MK) && args(newIrUnMod){
		irUnModSet = true;
		
		if(newIrUnMod == 0){
			bifFlag = true;
			bifTrigger = ChangeEvent.IR_UNMOD;
		}
			
		if(/*bifFlag ||*/ readyForStateCheck(MK)){
			System.out.println("***IR UN-MOD Changed BIF***");
			//StateCheck.GetState(ChangeEvent.IR_UNMOD, MK);
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	pointcut sonarChange_BIF(Kicker MK) : cflowbelow(ballInFrontPC(MK)) && set(float SensorControl.sonarRead)&& within(SensorControl);
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	after(Kicker MK, float newSonar):sonarChange_BIF(MK) && args(newSonar){
		sonarSet = true;
		
		if(newSonar <= SoccerGlobals.BALL_SONAR_DIST_GRAB){
			bifFlag = true;
			bifTrigger = ChangeEvent.SONAR;
		}
		
		if(bifFlag || readyForStateCheck(MK)){
			System.out.println("***Sonar Changed BIF***");
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	//****************************************************************************************************
	
	
	//Ball Close -- Cflow Section******************************************************************************
	
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	pointcut sonarChange_BC(Kicker MK) : cflowbelow(ballClosePC(MK)) && set(float SensorControl.sonarRead)&& within(SensorControl);
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	after(Kicker MK, float newSonar):sonarChange_BC(MK) && args(newSonar){
		sonarSet = true;
		
		if(newSonar <= SoccerGlobals.BALL_SONAR_DIST_GRAB){
			bifFlag = true;
			bifTrigger = ChangeEvent.SONAR;
		}
			
		if(bifFlag || readyForStateCheck(MK)){
			System.out.println("***Sonar Changed BC***");
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	//*****************************************************************************************************************************
	
	
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	pointcut sonarChange_bk(Kicker MK) : cflowbelow(ballKickablePC(MK)) && set(float SensorControl.sonarRead)&& within(SensorControl);
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	after(Kicker MK, float newSonar):sonarChange_bk(MK) && args(newSonar){
		sonarSet = true;
		
		if(newSonar <= SoccerGlobals.BALL_SONAR_DIST_GRAB){
			bifFlag = true;
			bifTrigger = ChangeEvent.SONAR;
		}
		
		if(bifFlag || readyForStateCheck(MK)){
			System.out.println("***Sonar Changed BK***");
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	//*****************************************************************************************************************************
	
	
	
	// Trigger needed events for after turn to ball state
	pointcut turnto_ball_state_exit(Kicker MK) : call(public boolean Kicker.turnToBall()) && this(MK);
	after(Kicker MK):turnto_ball_state_exit(MK){
		System.out.println("TURN Event EXIT");
		
		//************BIF FAIL TEST**************
		System.out.println("MOVE BALL FOR FAIL -- NOW!");
		Delay.msDelay(5000);
		System.out.println("Delay Finished!");
		//***************************************
		
		// Generate ballinfront_true event if the ball is in front
		if(MK.checkBallInFront(MK))
			MK.generateBallInFrontState();
	}
	
	
	// ARM**************************************
	pointcut openArmPC(MotionControl MC) : call(public void MotionControl.openArm(int)) && target(MC);
	after(MotionControl MC):openArmPC(MC){
		System.out.println("***Arm Open***");
		armOpen = true;
	}
	pointcut closeArmPC(MotionControl MC) : call(public void MotionControl.closeArm(int)) && target(MC);
	after(MotionControl MC):closeArmPC(MC){
		System.out.println("***Arm Close***");
		armOpen = false;
	}
	//******************************************

}