package org.usfirst.frc.team5895.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Arm {
	
	private enum Mode_Type {MOVING, BRAKING, DISENGAGING, PERCENT, DISABLED};
	private Mode_Type mode = Mode_Type.DISABLED;
	
	private TalonSRX armMaster;
	private VictorSPX armFollower1, armFollower2;
	private Solenoid brakeSolenoid;
	boolean brakeOn = false;
	
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 10;
	
	private double targetPos;
	private double footConversion = 9.22 * Math.pow(10, -5);
	private double carriageOffset = 0.54;
	private double brakeTimestamp;
	private double percentSetting;
	
	public Arm() {
		
		armMaster = new TalonSRX(1);
		armFollower1 = new VictorSPX(2);
		armFollower2 = new VictorSPX(3);
		
		armFollower1.follow(armMaster);
		armFollower2.follow(armMaster);
		
		brakeSolenoid = new Solenoid(1);
	
		/* first choose the sensor */
		armMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		armMaster.setSensorPhase(false);
		armMaster.setInverted(false);
		
		/* Set relevant frame periods to be at least as fast as periodic rate*/
		armMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		armMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		armMaster.configNominalOutputForward(0, kTimeoutMs);
		armMaster.configNominalOutputReverse(0, kTimeoutMs);
		armMaster.configPeakOutputForward(1, kTimeoutMs);
		armMaster.configPeakOutputReverse(-1, kTimeoutMs);
		
		/* Set current limiting */
		armMaster.configContinuousCurrentLimit(30, 0);
		armMaster.configPeakCurrentLimit(35, 0);
		armMaster.configPeakCurrentDuration(100, 0);
		armMaster.enableCurrentLimit(true);
		
		/* set closed loop gains in slot0 - see documentation */
		armMaster.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		armMaster.config_kF(0, 0.33, kTimeoutMs);
		armMaster.config_kP(0, 0.5, kTimeoutMs);
		armMaster.config_kI(0, 0, kTimeoutMs);
		armMaster.config_kD(0, 0, kTimeoutMs);
		
		/* set acceleration and vcruise velocity - see documentation */
		armMaster.configMotionCruiseVelocity(3900, kTimeoutMs);
		armMaster.configMotionAcceleration(7000, kTimeoutMs);
		
		/* zero the distance sensor */
		armMaster.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		
		/*set up the encoder */
	 	armMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		armMaster.setSelectedSensorPosition(0, 0, 10);
		
	}
	
	/* Motion Magic */
	/**
	 * sets the arm to go to a specified height 
	 * @param targetHeight the height to go to in feet
	 */
	public void setTargetPosition(double targetHeight) {
		targetPos = (targetHeight - carriageOffset) / footConversion;
		if(targetPos < 0.0) {
			targetPos = 0.0;
		}
		brakeTimestamp = Timer.getFPGATimestamp();
		mode = Mode_Type.DISENGAGING;
	}	
	
	/**
	 * gets the height of the arm in feet
	 * @return the height of the arm in feet
	 */
	public double getHeight() {
		
		return armMaster.getSelectedSensorPosition(0) * footConversion + carriageOffset; // 2048 ticks per rev, pitch diameter: 1.432in
	}
	
	/**
	 * checks if the arm is at the target position and moving slowly
	 * @return true if it is both at position and with low velocity, false otherwise
	 */
	public boolean atTarget() {
		return ((Math.abs(armMaster.getSelectedSensorPosition(0) - targetPos) < 200.0) 
				&& (Math.abs(armMaster.getSelectedSensorVelocity(0)) < 1.0));
	}
	
	public void update() {

		//this sets the max current based on if the limit switch is triggered
		if(!armMaster.getSensorCollection().isFwdLimitSwitchClosed()) {
//			DriverStation.reportError("forward limit switch triggered", false);
			armMaster.configPeakOutputForward(0.0, kTimeoutMs);	
		} else {
			armMaster.configPeakOutputForward(1.0, kTimeoutMs);
		}
		
		if (!armMaster.getSensorCollection().isRevLimitSwitchClosed()) {
//			DriverStation.reportError("reverse limit switch triggered", false);
			armMaster.configPeakOutputReverse(0.0, kTimeoutMs);	
		} else {
			armMaster.configPeakOutputReverse(-1.0, kTimeoutMs);
		}
		

		switch(mode) {
		
		case DISENGAGING:
			
			brakeOn = false;
			if(Timer.getFPGATimestamp() - brakeTimestamp > 0.1) {
				mode = Mode_Type.MOVING;
			}
			
			break;
		
		case MOVING:
			brakeOn = false;
			armMaster.set(ControlMode.MotionMagic, targetPos); 
			
			if(atTarget()) {
				mode = Mode_Type.BRAKING;
			}
			
			break;
			
		case BRAKING:
			
			brakeOn = true;
			armMaster.set(ControlMode.PercentOutput, 0);
			
			break;
			
		case PERCENT:
			brakeOn = false;
			armMaster.set(ControlMode.PercentOutput, percentSetting);
			
			break;
			
		case DISABLED:
			brakeOn = false;
			armMaster.set(ControlMode.PercentOutput, 0);
			break;
		}
		brakeSolenoid.set(brakeOn);
		
//		DriverStation.reportError("" + armMaster.getSensorCollection().isFwdLimitSwitchClosed(), false);
//		DriverStation.reportError("" + armMaster.getSelectedSensorPosition(0), false);
//		DriverStation.reportError("" + atTarget(), false);
//		DriverStation.reportError("" + targetPos, false);
//		System.out.println("" + getHeight());
		
	}
	
}
