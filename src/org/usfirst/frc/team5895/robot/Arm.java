package org.usfirst.frc.team5895.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

public class Arm {
	
	private enum Mode_Type {MOVING, BRAKING, DISENGAGING, PERCENT, DISABLED};
	private Mode_Type mode = Mode_Type.DISABLED;
	
	private TalonSRX armMaster;
	private Solenoid brakeSolenoid;
	boolean brakeOn = false;
	
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 10;
	
	private double targetPos;
	private double brakeTimestamp;
	private double percentSetting;
	
	private Potentiometer pot;
	
	public Arm() {
		
		armMaster = new TalonSRX(ElectricalLayout.MOTOR_ARM);
		
		brakeSolenoid = new Solenoid(ElectricalLayout.SOLENOID_ARM_BRAKE);
	
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
		
		pot = new AnalogPotentiometer(0, 180, 0);
		
	}
	
	/* Motion Magic */
	/**
	 * sets the arm to go to a specified position in degrees 
	 * @param target the position to go to in degrees
	 */
	private void setTargetPosition(double target) {
		targetPos = target;
		if (targetPos < 0)
			targetPos = 0;
		if (targetPos > 180)
			targetPos = 180;
		brakeTimestamp = Timer.getFPGATimestamp();
		mode = Mode_Type.DISENGAGING;
	}
	
	public void setToPosition1() {
		setTargetPosition(0);
	}
	
	public void setToPosition2() {
		setTargetPosition(45);
	}
	
	public void setToPosition3() {
		setTargetPosition(90);
	}
	
	public void setToPosition4() {
		setTargetPosition(135);
	}
	
	public void setToPosition5() {
		setTargetPosition(180);
	}
	
	/**
	 * @return the position of the arm based off of the Potentiometer.
	 */
	public double getPosition() {
		return pot.get();
	}
	
	/**
	 * checks if the arm is at the target position and moving slowly
	 * checks to see if the difference between the current position and the actual position is less than
	 * the threshold
	 * @return true if it is both at position and with low velocity, false otherwise
	 */
	public boolean atTarget() {
		int threshold = 2; // degrees
		return ((Math.abs(getPosition() - targetPos) < threshold) 
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
