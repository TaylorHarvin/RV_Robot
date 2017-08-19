package soccerControllers;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
//import lejos.robotics.RegulatedMotor;
import lejos.hardware.port.TachoMotorPort;
import lejos.hardware.sensor.HiTechnicCompass;
import lejos.robotics.DirectionFinderAdapter;
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

public class MotorController {
	private EV3LargeRegulatedMotor leftMotor;	// direct left motor controller
	private EV3LargeRegulatedMotor rightMotor;	// direct right motor controller
	private UnregulatedMotor arm;				// direct arm controller
	private Navigator roboNav;					// Primary navigator
	private MovePilot movePilot;
	private PoseProvider roboPos;
	
	
	private HiTechnicCompass compass;			// Compass for angles in navigator
	private float[] compassSamples;				// compass sample values
	private SampleProvider compassSP;			// compass sample provider
	private DirectionFinderAdapter compassDF;
	
	private float wheelDiam = (float) /*3.3*/4.746;
	private float trackWidth = (float) 6.6 /*7.6*/;
	
	
	private Wheel wheel1;
	private Wheel wheel2;
	private Chassis chassis;
	
	private CompassPoseProvider compassPose;
	private Waypoint lastGotoWayPoint;
	
	
	
	/* Constructor for the MotorController
	 * Parameters: 
	 * 	1. MotorPort leftMotorPort
	 * 		* The port for the left robot motor (A-D)
	 *  2. MotorPort rightMotorPort
	 * 		* The port for the right robot motor (A-D)
	 *  3. MotorPort arm
	 * 		* The port for the robot motor arm (A-D)
	 * Result:  All motors are initialized, unless incorrect ports are given
	 */
	public MotorController(Port leftMotorPort,Port rightMotorPort, Port armPort,Port compassPort){
		leftMotor = new EV3LargeRegulatedMotor(leftMotorPort);
		rightMotor = new EV3LargeRegulatedMotor(rightMotorPort);
		arm = new UnregulatedMotor(armPort);
		
		compass = new HiTechnicCompass(compassPort);
		compassSamples = new float[5];
		compassSP = compass.getAngleMode();
		compassDF = new DirectionFinderAdapter(compassSP);
		wheel1 = WheeledChassis.modelWheel(leftMotor, wheelDiam).offset(-1*trackWidth);
		wheel2 = WheeledChassis.modelWheel(rightMotor, wheelDiam /*1.62*/).offset(trackWidth);
		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		movePilot = new MovePilot(chassis);
		compassPose = new CompassPoseProvider(movePilot,compassDF);
		//roboNav = new Navigator(movePilot,compassPose);
		roboNav = new Navigator(movePilot);
		roboPos = roboNav.getPoseProvider();
		
		lastGotoWayPoint = new Waypoint(0,0,0);
	}
	
	
	
	public void GotoPoint(float xPos, float yPos, float heading, boolean blocking){
		boolean newPointGiven = false;
		Pose currPos = roboPos.getPose();
		
		if(lastGotoWayPoint.getX() != xPos || lastGotoWayPoint.getY() != yPos || lastGotoWayPoint.getHeading() != heading){
			newPointGiven = true;
		}
		else if(!roboNav.isMoving() && currPos.getX() != xPos && currPos.getY() != yPos && currPos.getHeading() != heading){
			newPointGiven = true;
		}
	
		if(newPointGiven){
			roboNav.clearPath();
			if(!Float.isNaN(heading)){
				//System.out.println("Heading Used");
				roboNav.goTo(xPos, yPos, heading);
			}
			else{
				//System.out.println("Heading Not Used");
				roboNav.goTo(xPos, yPos);
			}
			
			// Block return until the robot is at the requested spot if requested
			if(blocking){
				roboNav.waitForStop();
			}
		}
		
	}
	
	public void GoForward(double speed){
		SetForwardSpeed(speed);
		movePilot.forward();
	}
	
	public Pose GetPos(){
		return roboPos.getPose();
	}
		
	public void SetForwardSpeed(double speed){
		//System.out.println("Speed: "+speed);
		movePilot.setLinearSpeed(speed);
	}
	
	public void SetTurnSpeed(double speed){
		//System.out.println("Turn Speed: "+speed);
		movePilot.setAngularSpeed(speed);
	}
	
	public void Turn(double speed, float angle){
		SetTurnSpeed(speed);
		movePilot.rotate(angle);
	}
	
	public boolean RobotTurning(){
		float leftMotorSpeed = leftMotor.getRotationSpeed();
		float rightMotorSpeed = rightMotor.getRotationSpeed();
		
		if(leftMotorSpeed > 0 && rightMotorSpeed < 0)
			return true;
		else if(leftMotorSpeed < 0 && rightMotorSpeed > 0)
			return true;
		else
			return false;
	}
	
	public boolean RobotMoving(){
		return roboNav.isMoving();
	}
	
	
	public boolean InGoalRange(){
		double distToGoal = Math.sqrt(
				Math.pow(Globals.GOAL_LOCATION.getX() - GetRobotX(),2) +
				Math.pow(Globals.GOAL_LOCATION.getY() - GetRobotY(),2)
				);
		
		System.out.println("Goal Range: "+ distToGoal);
		if(distToGoal <= Globals.GOAL_RANGE_THRESHOLD)
			return true;
		
		return false;
		
	}
	
	public boolean IsSamePos(Waypoint otherPoint){
		Pose currPos = GetPos();
		return otherPoint.x == currPos.getX() && otherPoint.y == currPos.getY();
	}
	
	public boolean InRange(Waypoint refPoint, float thresDist){
		Pose currPos = GetPos();
		return refPoint.distance(currPos.getX(), currPos.getY()) <= thresDist;
	}
	
	public void MoveArm(float[] armPosSeq, int speed){
		arm.setPower(speed);
		for(int i = 0; i < armPosSeq.length;i+=2){
			if(armPosSeq[i] == 0)
				arm.forward();
			else
				arm.backward();
			Delay.msDelay((long) armPosSeq[i+1]);
		}
	}
	
	public void Stop(){
		roboNav.clearPath();
		roboNav.stop();
	}
	
	
	
	
	public float GetRobotX(){
		return GetPos().getX();
	}
	
	public float GetRobotY(){
		return GetPos().getY();
	}
	
	public float GetRobotHeading(){
		return GetPos().getHeading();
	}
	
	
}
