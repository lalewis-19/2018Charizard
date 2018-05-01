package org.usfirst.frc.team5895.robot;

import org.usfirst.frc.team5895.robot.framework.Looper;
import org.usfirst.frc.team5895.robot.lib.BetterJoystick;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	Looper loop;
	
	// Subsystems
	Arm arm;
	ClawHat intake;
	Drivetrain drive;
	
	// joysticks
	BetterJoystick leftJoystick;
	BetterJoystick rightJoystick;
	
	public void robotInit() {
		arm = new Arm();
		intake = new ClawHat();
		drive = new Drivetrain();
		
		leftJoystick = new BetterJoystick(0);
		rightJoystick = new BetterJoystick(1);
		
		loop = new Looper(10);
		loop.add(arm::update);
		loop.add(intake::update);
		loop.add(drive::update);
		
		loop.start();
	}

	public void autonomousInit() {
	
	}

	public void teleopPeriodic() {
		//teleop drive
		drive.arcadeDrive(leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(0));
		
		if(leftJoystick.getRisingEdge(1)) {
			// intake -> hat left
		} else if(leftJoystick.getRisingEdge(2)){
			arm.setToDown();
		} else if(leftJoystick.getRisingEdge(2)){
			arm.setTo45();
		} 
		
		if(rightJoystick.getRisingEdge(1)) {
			// intake -> hat right
		} else if(rightJoystick.getRisingEdge(2)) {
			arm.setToUp();
		}
	}

	public void testPeriodic() {
	
	}
}
