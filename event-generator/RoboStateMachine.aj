/* Developer: Taylor Harvin
 * Date Last Changed: 8/14/2016
 * Purpose: Provide state properties of the robot
 *
 */

import soccerControllers.*;
import soccerPlayers.*;
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
	
	
	// Enough information is set to do a valid state check
	public boolean readyForStateCheck(Kicker MK){
		// All sensor values are set -- allow state check
		// Otherwise -- look at the 
		if(sonarSet && irModSet && irUnModSet)
			return true;
		else if(sonarSet && MK.GetSensorController().GetLastSonar() <= Globals.SONAR_IN_ARM_READING)
			return true;
		else if(irModSet && MK.GetSensorController().GetLastModIR() == 0)
			return true;
		else if(irUnModSet && MK.GetSensorController().GetLastUnModIR() == 0)
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
		//System.out.println("Generated BIF");
	}
	
	// Check the current ball in front state from the Kicker (without triggering any events)
	public boolean Kicker.checkBallInFront(Kicker currMK){
		return currMK.BallInFront();
	}
	
	
	// Generate the ball close true event on-demand rather than through Kicker
	public void Kicker.generateBallCloseState(){
		//System.out.println("Generated Ball Close");
	}
	
	// Check the current ball close state from the Kicker (without triggering any events)
	public boolean Kicker.checkBallClose(Kicker currMK){
		return currMK.BallClose();
	}
	
	
	// Generate the current state on-demand here rather than directly from state check method
	public void Kicker.generateStateEvent(State currState, Kicker currMK){
		switch(currState){
			case TURN_TO_BALL:
				//System.out.println("GEN TURN_TO_BALL");
				// NOTE: Ensure that wonder event is triggered before BIF
				StateCheck.TurnToBallState(currMK);
				break;
			case GOTO_BALL:
				//System.out.println("GEN GOTO_BALL");
				StateCheck.GotoBallState(currMK);
				break;
			case TURN_TO_GOAL:
				//System.out.println("GEN TURN_TO_GOAL");
				StateCheck.TurnToGoalState(currMK);
				break;
			case DRIBBLE_TO_GOAL:
				//System.out.println("GEN DRIBBLE_TO_GOAL");
				StateCheck.DribbleBallState(currMK);
				break;
			case KICK_BALL_TO_GOAL:
				//System.out.println("GEN KICK_BALL_TO_GOAL");
				StateCheck.KickBallAtGoal(currMK);
				break;
			default:
				//System.out.println(currState);
				//System.out.println("GEN NONE");
				break;
		}
		
	}
	
	
	
	
	
	
	//POINTCUT SECTION**************************************************************************
	// General pointcut to allow for access to Kicker object for other pointcuts (through cflowbelow)
	pointcut PlayPC(Kicker MK) : call(public void Kicker.Play()) && target(MK);
	pointcut BallInFrontPC(Kicker MK) : call(public boolean Kicker.BallInFront()) && target(MK);
	pointcut ballClosePC(Kicker MK) : call(public boolean Kicker.BallClose()) && target(MK);
	pointcut ballKickablePC(Kicker MK) : call(public boolean Kicker.ballKickable(boolean)) && target(MK);
	//ADVICE SECTION*****************************************************************************
	
	//BIF -- Cflow Section******************************************************************************
	// IR -- Mod advice, handle change in IR MOD value (after new ping)
	pointcut irModChange_BIF(Kicker MK) : cflowbelow(BallInFrontPC(MK)) && set(float SensorController.ballDirMod)&& within(SensorController);
	// IR -- Mod advice, handle change in IR MOD value (after new ping)
	after(Kicker MK, float newIrMod) :irModChange_BIF(MK) && args(newIrMod){
		irModSet = true;
		
		
		if(newIrMod == 0){
			bifFlag = true;
			bifTrigger = ChangeEvent.IR_MOD;
		}
		
		if(/*bifFlag ||*/ readyForStateCheck(MK)){
			//System.out.println("***IR MOD Changed BIF***");
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	
	
	// IR -- Un-Mod advice, handle change in IR UN-MOD value (after new ping)
	pointcut irUnModChange_BIF(Kicker MK) : cflowbelow(BallInFrontPC(MK)) && set(float SensorController.ballDirUnMod)&& within(SensorController);
	// IR -- Un-Mod advice, handle change in IR UN-MOD value (after new ping)
	after(Kicker MK, float newIrUnMod):irUnModChange_BIF(MK) && args(newIrUnMod){
		irUnModSet = true;
		
		if(newIrUnMod == 0){
			bifFlag = true;
			bifTrigger = ChangeEvent.IR_UNMOD;
		}
			
		if(/*bifFlag ||*/ readyForStateCheck(MK)){
			//System.out.println("***IR UN-MOD Changed BIF***");
			//StateCheck.GetState(ChangeEvent.IR_UNMOD, MK);
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	pointcut sonarChange_BIF(Kicker MK) : cflowbelow(BallInFrontPC(MK)) && set(float SensorController.sonarRead)&& within(SensorController);
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	after(Kicker MK, float newSonar):sonarChange_BIF(MK) && args(newSonar){
		sonarSet = true;
		
		if(newSonar <= Globals.SONAR_IN_ARM_READING){
			bifFlag = true;
			bifTrigger = ChangeEvent.SONAR;
		}
		
		if(bifFlag || readyForStateCheck(MK)){
			//System.out.println("***Sonar Changed BIF***");
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	//****************************************************************************************************
	
	
	//Ball Close -- Cflow Section******************************************************************************
	
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	pointcut sonarChange_BC(Kicker MK) : cflowbelow(ballClosePC(MK)) && set(float SensorController.sonarRead)&& within(SensorController);
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	after(Kicker MK, float newSonar):sonarChange_BC(MK) && args(newSonar){
		sonarSet = true;
		
		if(newSonar <= Globals.SONAR_IN_ARM_READING){
			bifFlag = true;
			bifTrigger = ChangeEvent.SONAR;
		}
			
		if(bifFlag || readyForStateCheck(MK)){
			//System.out.println("***Sonar Changed BC***");
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	//*****************************************************************************************************************************
	
	
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	pointcut sonarChange_bk(Kicker MK) : cflowbelow(ballKickablePC(MK)) && set(float SensorController.sonarRead)&& within(SensorController);
	// Sonar -- Sonar advice, handle change in Sonar value (after new ping)
	after(Kicker MK, float newSonar):sonarChange_bk(MK) && args(newSonar){
		sonarSet = true;
		
		if(newSonar <= Globals.SONAR_IN_ARM_READING){
			bifFlag = true;
			bifTrigger = ChangeEvent.SONAR;
		}
		
		if(bifFlag || readyForStateCheck(MK)){
			//System.out.println("***Sonar Changed BK***");
			State currState = StateCheck.GetState(bifTrigger, MK);
			MK.generateStateEvent(currState,MK);
			StateCheck.PrintState(currState);
			resetStatePreCheck();
		}
	}
	//*****************************************************************************************************************************
	
	
	
	// Trigger needed events for after turn to ball state
	pointcut turnto_ball_state_exit(Kicker MK) : call(public boolean Kicker.TurnToBall()) && this(MK);
	after(Kicker MK):turnto_ball_state_exit(MK){
		//System.out.println("TURN Event EXIT");
		
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
	/*pointcut openArmPC(MotorController MC) : call(public void MotorController.openArm(int)) && target(MC);
	after(MotorController MC):openArmPC(MC){
		System.out.println("***Arm Open***");
		armOpen = true;
	}
	pointcut closeArmPC(MotorController MC) : call(public void MotorController.closeArm(int)) && target(MC);
	after(MotorController MC):closeArmPC(MC){
		System.out.println("***Arm Close***");
		armOpen = false;
	}*/
	//******************************************

}