package org.usfirst.frc.team5895.robot;

import org.usfirst.frc.team5895.robot.lib.PathfinderFollower;
import org.usfirst.frc.team5895.robot.lib.TrajectoryDriveController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.usfirst.frc.team5895.robot.lib.NavX;
import org.usfirst.frc.team5895.robot.lib.PID;
import org.usfirst.frc.team5895.robot.lib.BetterEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

public class Drivetrain {

	private TalonSRX leftDriveMaster, rightDriveMaster;
	private VictorSPX leftDriveFollower1, leftDriveFollower2, rightDriveFollower1, rightDriveFollower2;
	private BetterEncoder leftEncoder,rightEncoder;
	private double leftspeed, rightspeed;
	private NavX navX;

	private TrajectoryDriveController pInUse;
	
	private enum Mode_Type {TELEOP,AUTO_SPLINE, AUTO_BACKWARDS_SPLINE, AUTO_MIRROR_SPLINE, AUTO_MIRROR_BACKWARDS_SPLINE, AUTO_DRIVE};
	private Mode_Type mode = Mode_Type.TELEOP;
	
	private PID turnPID;
	private boolean turning;
	private PID drivePID;
	
	public static final double TURN_P = 0.030; 
	public static final double TURN_I = 0.00000001;
	private static final double capSpeed = 0.5;
	
	private static final double DRIVE_KP = 0.1;
	private static final double DRIVE_KI = 0.00000;
	
	// tracking
	private double posX, posY; // feet
	private double lastDistance = 0d; // distance traveled the last time update() was called
	
	public Drivetrain() {
	
		//initialize drive motors
		leftDriveMaster = new TalonSRX(ElectricalLayout.MOTOR_DRIVE_LEFT_MASTER);
		leftDriveFollower1 = new VictorSPX(ElectricalLayout.MOTOR_DRIVE_LEFT_FOLLOWER_1);
		leftDriveFollower2 = new VictorSPX(ElectricalLayout.MOTOR_DRIVE_LEFT_FOLLOWER_2);
		rightDriveMaster = new TalonSRX(ElectricalLayout.MOTOR_DRIVE_RIGHT_MASTER);
		rightDriveFollower1 = new VictorSPX(ElectricalLayout.MOTOR_DRIVE_RIGHT_FOLLOWER_1);
		rightDriveFollower2 = new VictorSPX(ElectricalLayout.MOTOR_DRIVE_RIGHT_FOLLOWER_2);
			
		//initialize encoders
		leftEncoder = new BetterEncoder(ElectricalLayout.ENCODER_DRIVE_LEFT_1, ElectricalLayout.ENCODER_DRIVE_LEFT_2, true, Encoder.EncodingType.k4X);
		rightEncoder = new BetterEncoder(ElectricalLayout.ENCODER_DRIVE_RIGHT_1, ElectricalLayout.ENCODER_DRIVE_RIGHT_2, false, Encoder.EncodingType.k4X);
		
		//set encoder distance per pulse so it's in feet
		leftEncoder.setDistancePerPulse(6/12.0*Math.PI/360); //correct
		rightEncoder.setDistancePerPulse(6/12.0*Math.PI/360); //correct
		
		//initialize navx
		navX = new NavX();
		
		// initialize PID
		turnPID = new PID(TURN_P, TURN_I, 0, 6, true, 1.0, true);
		drivePID = new PID(DRIVE_KP, DRIVE_KI, 0, 6);
		
		//initializes trajectory generator from auto files
		//IF ONE ISN'T ON THE ROBOT ALL THE ONES AFTER IT WON'T WORK
		try { 
			// drive straight
//			pStraight = new TrajectoryDriveController("/home/lvuser/AutoFiles/Straight.txt", 0.0, 0, 0, 1.0/11.5, 1.0/50.0, -0.05);
			
		}catch (Exception e) {
			DriverStation.reportError("All auto files not on robot!", false);
		}
	}
	
	/**
	 * resets the encoder values to 0
	 */
	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
		
	}
	
	/**
	 * turns the robot to a certain angle using the PID.
	 * @param angle the angle that the robot should point to.
	 */
	public void turnTo(double angle) {
		turnPID.set(angle);
		turning = true;
		mode = Mode_Type.TELEOP;
	}
	
	/**
	 * makes the robot stop turning using the PID
	 */
	public void stopTurning() {
		turning = false;
	}
	
	/**
	 * returns if the robot is turning to an angle using the PID
	 * @return if the robot is turning to an angle using the PID
	 */
	public boolean isTurning() {
		return turning;	
	}
	
	/**
	 * resets the NavX to 0
	 */
	public void resetNavX() {
		navX.reset();
	}
	
	/**
	 * gets the current angle of the NavX
	 * @return the NavX angle in degrees
	 */
	public double getAngle() {
		return navX.getAngle(); // navX reads angle in degree
	}

	
	/**
	 * normal arcade drive code for teleop use
	 * @param speed the forward speed to go at
	 * @param turn the speed to turn at
	 */
	public void arcadeDrive(double speed, double turn) {
		leftspeed = speed - turn;
		rightspeed = speed + turn;
		mode = Mode_Type.TELEOP;
		turning = false;
	}
	
	/**
	 * @return the average velocity in feet per second from the left and right encoders.
	 */
	public double getVelocity() {
		return (leftEncoder.getRate()+rightEncoder.getRate())/2;
	}
	
	/**
	 * @return the average distance traveled in feet from the left and right encoders.
	 */
	public double getDistanceTraveled() {
		return (leftEncoder.getDistance()+rightEncoder.getDistance())/2;
	}
	
	/**
	 * gets the x position of the Drivetrain
	 * @return the x position of the Drivetrain in feet
	 */
	public double getXPosition() {
		return posX;
	}
	
	/**
	 * gets the y position of the Drivetrain
	 * @return the y position of the Drivetrain in feet
	 */
	public double getYPosition() {
		return posY;
	}
	
	/**
	 * checks whether the current path is finished
	 * @return true if the path is finished, false otherwise
	 */
	public boolean isPFinished() {
		return (pInUse.isFinished());
	}
	
	/**
	 * stop turnPID when robot is at correct angle
	 * @return whether PID is at correct angle 
	 */
	public boolean atAngle() {
		return (Math.abs(turnPID.getSetpoint() - getAngle()) <= 5);
	}
	
	/**
	 * stop drivePID when robot is at correct distance
	 * @return whether PID is at correct distance 
	 */
	public boolean atDistance() {
		return (Math.abs(drivePID.getSetpoint() - getDistanceTraveled()) <= 2);
	}
	
	public void update() {
		
		//changing position and distance
		double distance = getDistanceTraveled()-lastDistance;
		
		posX += distance*Math.cos(Math.toRadians(getAngle()));
		posY += distance*Math.sin(Math.toRadians(getAngle()));
		
		distance = getDistanceTraveled();
		
		switch(mode) {
			
			//spline driving with the claw forward
			case AUTO_SPLINE:
				double[] m = new double[2];
				m = pInUse.getOutput(rightEncoder.getDistance(), leftEncoder.getDistance(), getAngle()*3.14/180); 

				leftDriveMaster.set(ControlMode.PercentOutput, m[1]);
				rightDriveMaster.set(ControlMode.PercentOutput, -m[0]);
				leftDriveFollower1.set(ControlMode.PercentOutput, m[1]);
				rightDriveFollower1.set(ControlMode.PercentOutput, -m[0]);
				leftDriveFollower2.set(ControlMode.PercentOutput, m[1]);
				rightDriveFollower2.set(ControlMode.PercentOutput, -m[0]);
				break;
				
			//spline driving with the claw backward
			case AUTO_BACKWARDS_SPLINE:
				double[] m_back = new double[2];
				m_back = pInUse.getOutput(-leftEncoder.getDistance(), -rightEncoder.getDistance(), getAngle()*3.14/180); 

				leftDriveMaster.set(ControlMode.PercentOutput, -m_back[0]);
				rightDriveMaster.set(ControlMode.PercentOutput, m_back[1]);
				leftDriveFollower1.set(ControlMode.PercentOutput, -m_back[0]);
				rightDriveFollower1.set(ControlMode.PercentOutput, m_back[1]);
				leftDriveFollower2.set(ControlMode.PercentOutput, -m_back[0]);
				rightDriveFollower2.set(ControlMode.PercentOutput, m_back[1]);
				break;
			
			//mirrored spline driving for the other side of the field 
			//with the claw forward
			case AUTO_MIRROR_SPLINE:
				double[] m_mirror = new double[2];
				m_mirror = pInUse.getOutput(leftEncoder.getDistance(), rightEncoder.getDistance(), -getAngle()*3.14/180); 

				leftDriveMaster.set(ControlMode.PercentOutput, m_mirror[0]);
				rightDriveMaster.set(ControlMode.PercentOutput, -m_mirror[1]);
				leftDriveFollower1.set(ControlMode.PercentOutput, m_mirror[0]);
				rightDriveFollower1.set(ControlMode.PercentOutput, -m_mirror[1]);
				leftDriveFollower2.set(ControlMode.PercentOutput, m_mirror[0]);
				rightDriveFollower2.set(ControlMode.PercentOutput, -m_mirror[1]);
				break;
				
			//mirrored spline driving for the other side of the field
			//with the claw backward
			case AUTO_MIRROR_BACKWARDS_SPLINE: 
				double[] m_mirror_back = new double[2];
				m_mirror_back = pInUse.getOutput(-rightEncoder.getDistance(), -leftEncoder.getDistance(), -getAngle()*3.14/180); 
				
				leftDriveMaster.set(ControlMode.PercentOutput, -m_mirror_back[1]);
				rightDriveMaster.set(ControlMode.PercentOutput, m_mirror_back[0]);
				leftDriveFollower1.set(ControlMode.PercentOutput, -m_mirror_back[1]);
				rightDriveFollower1.set(ControlMode.PercentOutput, m_mirror_back[0]);
				leftDriveFollower2.set(ControlMode.PercentOutput, -m_mirror_back[1]);
				rightDriveFollower2.set(ControlMode.PercentOutput, m_mirror_back[0]);
				break;
			
			//drive straight with PID	
			case AUTO_DRIVE:
				leftspeed = drivePID.getOutput(getDistanceTraveled());
				rightspeed = -drivePID.getOutput(getDistanceTraveled());
				leftDriveMaster.set(ControlMode.PercentOutput, leftspeed);
				rightDriveMaster.set(ControlMode.PercentOutput, rightspeed);
				leftDriveFollower1.set(ControlMode.PercentOutput, leftspeed);
				rightDriveFollower1.set(ControlMode.PercentOutput, rightspeed);
				leftDriveFollower2.set(ControlMode.PercentOutput, leftspeed);
				rightDriveFollower2.set(ControlMode.PercentOutput, rightspeed);
				break;	
			
			
			//teleop driving with joystick control
			case TELEOP: 
				if (turning) {
					leftspeed = -turnPID.getOutput(navX.getAngle());
					rightspeed = turnPID.getOutput(navX.getAngle());
					
					if(leftspeed > capSpeed) {
						leftspeed = capSpeed;
					}
					if(rightspeed > capSpeed) {
						rightspeed = capSpeed;
					}
					if(leftspeed < -capSpeed) {
						leftspeed = -capSpeed;
					}
					if(rightspeed < -capSpeed) {
						rightspeed = -capSpeed;
					}
				}
				leftDriveMaster.set(ControlMode.PercentOutput, -leftspeed);
				rightDriveMaster.set(ControlMode.PercentOutput, rightspeed);
				leftDriveFollower1.set(ControlMode.PercentOutput, -leftspeed);
				rightDriveFollower1.set(ControlMode.PercentOutput, rightspeed);
				leftDriveFollower2.set(ControlMode.PercentOutput, -leftspeed);
				rightDriveFollower2.set(ControlMode.PercentOutput, rightspeed);
				break;
			
		}
	}

}