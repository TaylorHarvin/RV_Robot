package soccerPlayers;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.HiTechnicCompass;
import lejos.robotics.DirectionFinderAdapter;
import lejos.robotics.Gyroscope;
import lejos.robotics.GyroscopeAdapter;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.CompassPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;
import lejos.utility.GyroDirectionFinder;
import soccerControllers.Globals;
import soccerControllers.MotorController;
import soccerControllers.SensorController;

// NEW KICKER!!!
public class Kicker {
	private MotorController mainMC;
	private  SensorController mainSC;
	private float[] currBallIR;
	private float currBallSonar;
	private boolean continuePlaying = true;
	
	public Kicker(){
		mainMC = new MotorController(MotorPort.A,MotorPort.D, MotorPort.C, SensorPort.S1);
		mainSC = new SensorController(SensorPort.S3,SensorPort.S2);
		currBallIR = new float[2];
		mainSC.FlushSensors();
	}
	
	public  MotorController GetMotorController(){
		return mainMC;
	}
	
	public  SensorController GetSensorController(){
		return mainSC;
	}
	
	
	public void Play(){
		System.out.println("PLAY");
		boolean ballKickedAtGoal = false;
		boolean tmp = false;
		/*while(!GotoBall())
			System.out.println("TURN RES :"+this.TurnToBall());*/
		/*while(!tmp){
			tmp = FindBall();
			System.out.println(tmp);
			Delay.msDelay(2000);
			//System.out.println("Ball Find FAIL!!!");
		}
		*/
		
		this.GotoGoal(false);
		/*while(true){
			System.out.println("BIF:"+BallInFront());
		}*/
		//GotoGoal(false);
		
		
		
		/*while(!ballKickedAtGoal){
			if(FindBall()){
				if(GotoGoal(true)){
					// Kick Ball
					ballKickedAtGoal = true;
					//System.out.println("Ball Kicked at goal!");
				}
				else{
					//System.out.println("Lost BALL!");
				}
			}
		}*/
	}
	
	
	
	// Wonders for the ball until it has the ball in it's arms
	public  boolean FindBall(){
		System.out.println("IN-FIND-BAll");
		int findTrys = 0;
		do{
			TurnToBall();
			findTrys++;
			if(findTrys > Globals.FIND_BALL_TRY_MAX)
				return false;
		}while(!GotoBall());
		
		return true;
	}
	
	public  boolean TurnToBall(){
		Pose currPos;
		float avgBallAngle = 0;
		float lastBallAngle = 0;
		float currTurnSpeed = 0;
		int newTurnCount = 0;
		
		boolean bif = false;
		
		if(BallInFront()){
			System.out.println("TEST!!!");
			return true;
		}
			
		do{
			currPos = mainMC.GetPos();
			avgBallAngle = GetBallAngle();
			if(avgBallAngle < 0)
				avgBallAngle -= 30.0;
			else if(avgBallAngle > 0)
				avgBallAngle += 30.0;
				
			//currTurnSpeed = (float) Math.abs(((avgBallAngle/360.0)*Globals.MAX_MOTOR_SPEED*2) + 50);
			currTurnSpeed = 25;
			mainMC.SetTurnSpeed(currTurnSpeed);
			if(lastBallAngle != avgBallAngle || newTurnCount >= Globals.TURN_FAIL_SAFE_LIMIT){
				mainMC.Turn(currTurnSpeed, avgBallAngle*-1);
				//mainMC.GotoPoint(currPos.getX(), currPos.getY(), currPos.getHeading()+avgBallAngle, false);
				newTurnCount = 0;
			}
			
			
			lastBallAngle = avgBallAngle;
			newTurnCount++;
			
			System.out.println("BIF:"+BallInFront());
			//Delay.msDelay(5000);
		}while(!BallInFront());
		
		//System.out.println("!!! TURN: "+BallInFront());
		return true;
	}
	
	public  float GetBallAngle(){
		float finalAngle = (float) 0.0;
		int numAngleVals = 0;
		
		currBallIR[Globals.IR_UNMOD] = mainSC.GetIR(Globals.IR_UNMOD);
		currBallIR[Globals.IR_MOD] = mainSC.GetIR(Globals.IR_MOD);
		
		if(!Float.isNaN(currBallIR[Globals.IR_MOD])){
			finalAngle += currBallIR[Globals.IR_MOD];
			numAngleVals++;
			/*if(currBallIR[Globals.IR_MOD] == 0)
				return currBallIR[Globals.IR_MOD];*/
		}
		if(!Float.isNaN(currBallIR[Globals.IR_UNMOD])){
			finalAngle += currBallIR[Globals.IR_UNMOD];
			numAngleVals++;
			/*if(currBallIR[Globals.IR_UNMOD] == 0)
				return currBallIR[Globals.IR_UNMOD];*/
		}
		
		// No valid angle returned, use a default 180
		// to try to get the ball in range of IR
		if(numAngleVals == 0){
			finalAngle = 180;
			numAngleVals = 1;
		}
		
		/*if(finalAngle <= 0)
			finalAngle += Globals.BUFFER_ANGLE;
		else
			finalAngle -= Globals.BUFFER_ANGLE;*/
		
		return (float) (finalAngle/2.0);
	}
	
	
	public  boolean BallInFront(){
		currBallIR[Globals.IR_UNMOD] = mainSC.GetIR(Globals.IR_UNMOD);
		currBallIR[Globals.IR_MOD] = mainSC.GetIR(Globals.IR_MOD);
		currBallSonar = mainSC.GetSonar();
		
		//System.out.println("Ball Close: "+BallClose());
		System.out.println(currBallIR[Globals.IR_UNMOD] +" , "+currBallIR[Globals.IR_MOD]+" , "+currBallSonar);
		
		if(currBallIR[Globals.IR_UNMOD] == 0 || currBallIR[Globals.IR_MOD] == 0)
			return true;
		else if(BallClose() /*&& BallInBufferAngleRange()*/)
			return true;
		else
			return false;
	}
	
	public  boolean BallClose(){
		currBallSonar = mainSC.GetSonar();
		return (currBallSonar <= Globals.SONAR_CLOSE_READING && currBallSonar != Globals.SONAR_ERROR_VAL);
	}
	
	public  boolean HasBall(){
		currBallSonar = mainSC.GetSonar();
		//System.out.println("Has Ball Sonar: "+currBallSonar);
		return (currBallSonar <= Globals.SONAR_IN_ARM_READING);
	}
	
	public  boolean GotoBall(){
		float lastSonar = 0;
		currBallSonar = mainSC.GetSonar();
		lastSonar = currBallSonar;
		if(currBallSonar != Globals.SONAR_ERROR_VAL && !Float.isInfinite(currBallSonar))
			//mainMC.GoForward(currBallSonar*Globals.MAX_MOTOR_SPEED);
			mainMC.GoForward(7);
		else
			//mainMC.GoForward(Globals.MAX_MOTOR_SPEED/10);
			mainMC.GoForward(7);
		
		do{
			currBallSonar = mainSC.GetSonar();
			if(currBallSonar < lastSonar && !Float.isInfinite(currBallSonar) && currBallSonar != Globals.SONAR_ERROR_VAL)
				mainMC.SetForwardSpeed((float)(((currBallSonar/360.0)*Globals.MAX_MOTOR_SPEED) + 1));
			lastSonar = currBallSonar;
			//System.out.println("In Goto: "+(((currBallSonar/360.0)*Globals.MAX_MOTOR_SPEED) + 1));
		}while(BallInFront() && !HasBall());
		
		//System.out.println("In Goto -- Has Ball: "+HasBall());
		return HasBall();
	}
	
	public  boolean BallInBufferAngleRange(){
		float avgBallAngle = GetBallAngle();
		
		return (avgBallAngle >= (-1.0*Globals.BUFFER_ANGLE) && avgBallAngle <= Globals.BUFFER_ANGLE);
	}
	
	public  boolean GotoGoal(boolean withBall){
		//System.out.println("IN-GOTO-GOAL");
		mainMC.SetForwardSpeed(Globals.MAX_MOTOR_SPEED/2);
		mainMC.SetTurnSpeed(Globals.MAX_MOTOR_SPEED/2);
		Waypoint goalLoc = Globals.GOAL_LOCATION;
		mainMC.GotoPoint(goalLoc.x, goalLoc.y, Float.NaN, false);
		while(!mainMC.InGoalRange()){
			if(withBall){
				if(!BallInFront()){
					System.out.println("!!!Ball not in front for Goal!!!");
					return false;
				}
				if(BallClose()){
					//Dribble
				}
			}
		}
		return true;
	}
	
	
	public  void KickBall(){
		float[] kickBallSeq = {0,1000,1,1000};
		mainMC.GoForward(Globals.MAX_MOTOR_SPEED);
		Delay.msDelay(2000);
		mainMC.MoveArm(kickBallSeq, 100);
		mainMC.Stop();
	}
}
