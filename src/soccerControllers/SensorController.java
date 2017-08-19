package soccerControllers;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.HiTechnicIRSeekerV2;
import lejos.robotics.SampleProvider;

public class SensorController {
	private EV3UltrasonicSensor sonarSensor;	// Actual Sonar sensor
	private SampleProvider sonarSP;				// Sonar Data Grabber
	private float[] sonarSamples;				// Actual sonar values
	
	private HiTechnicIRSeekerV2 irSensor;		// Actual IR Seeker
	private SampleProvider irSP[];				// IR Data Grabber: 0 -> unMod, 1 -> Mod
	private float[] irSamples;					// Actual IR values
	
	private float ballDirMod = 0;
	private float ballDirUnMod = 0;
	private float sonarRead = 0;
	
	
	public SensorController(Port irPort, Port sonarPort){
		sonarSensor = new EV3UltrasonicSensor(sonarPort);
		sonarSP = sonarSensor.getDistanceMode();
		sonarSamples = new float[5];
		
		irSensor = new HiTechnicIRSeekerV2(irPort);
		irSP = new SampleProvider[2];
		irSP[0] = irSensor.getUnmodulatedMode();
		irSP[1] = irSensor.getModulatedMode();
		irSamples = new float[5];
	}
	
	public float GetIR(int mode){
		irSP[mode].fetchSample(irSamples, 0);
		if(mode == Globals.IR_UNMOD)
			this.ballDirUnMod = irSamples[0];
		else
			this.ballDirMod = irSamples[0];
		return irSamples[0];
	}
	
	public float GetSonar(){
		sonarSP.fetchSample(sonarSamples, 0);
		this.sonarRead = sonarSamples[0];
		return sonarSamples[0];
	}
	
	public float GetLastSonar(){
		return this.sonarRead;
	}
	
	public float GetLastModIR(){
		return this.ballDirMod;
	}
	
	public float GetLastUnModIR(){
		return this.ballDirUnMod;
	}
	
	
	public void FlushSensors(){
		System.out.println("-----Flushing Sensors Start-----");
		if(Float.isNaN(GetIR(Globals.IR_MOD))){
			System.out.println("IR_MOD -- NO BALL");
		}
		if(Float.isNaN(GetIR(Globals.IR_UNMOD))){
			System.out.println("IR_UNMOD -- NO BALL");
		}
		if(GetSonar() == Globals.SONAR_ERROR_VAL){
			System.out.println("TMP Sonar Error");
		}
		System.out.println("-----Flushing Sensors End-----");
	}
}
