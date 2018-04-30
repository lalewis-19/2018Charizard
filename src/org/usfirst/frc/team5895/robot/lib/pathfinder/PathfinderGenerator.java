package org.usfirst.frc.team5895.robot.lib.pathfinder;

import java.io.File;
import edu.wpi.first.wpilibj.DriverStation;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * FitMethod: HERMITE.CUBIC or HERMITE.QUINTIC
 * SAMPLE: HIGH (100 000 points), LOW (10 000 points), FAST (1 000)
 *
 */

public class PathfinderGenerator {
	private boolean saveCSV;
	private final String CSVLocation = "/home/lvuser/PathFinderFiles/"; 
	
	public PathfinderGenerator(boolean saveCSV){
		this.saveCSV = saveCSV;
	}
	
	public PathfinderGenerator() {
		this(false);
	}
	
	public void makeCSV(Trajectory trajectory, String name) {
		if(saveCSV) {
			File myFile = new File(CSVLocation + name + ".csv");
			Pathfinder.writeToCSV(myFile, trajectory);
		}
	}
	
	public double d2r( double degree) {
		return Pathfinder.d2r(degree);
	}
	
	public Trajectory Straight() {
		String name = "Straight";
		double dt = 0.01; // in second
		double max_vel = 5; // in f/s
		double max_acc = 10; // in f/s/s
		double max_jer = 60.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 0, d2r(0)),     
    		    new Waypoint(5, 0, d2r(0)),                        
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	
    	return trajectory;		
	}
	
	public Trajectory RightLeftScale() {
		String name = "RightLeftScale";
		double dt = 0.01; // in second
		double max_vel = 8; // in f/s
		double max_acc = 10; // in f/s/s
		double max_jer = 60.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 0, d2r(0)),     
    		    new Waypoint(10, 0, d2r(0)), 
    		    new Waypoint(16, 10, d2r(89.0)),     
    		    new Waypoint(16.0, 14, d2r(89.0)),
    		    new Waypoint(18.0, 16.0, d2r(0)),     
    		    new Waypoint(21.5, 15.5, d2r(-30.0)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory RightRightScale() {
		String name = "RightRightScale";
		double dt = 0.01; // in second
		double max_vel = 6; // in f/s
		double max_acc = 11; // in f/s/s
		double max_jer = 60.0; // in f/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 0, d2r(0)),     
    		    new Waypoint(16, 0, d2r(0)), 
    		    new Waypoint(20, 1, d2r(30.0)),     
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory CenterRightSwitchFront() {
		String name = "CenterRightSwitchFront";
		double dt = 0.01; // in second
		double max_vel = 6; // in f/s
		double max_acc = 11; // in f/s/s
		double max_jer = 60.0; // in f/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 0, d2r(0)),     
    		    new Waypoint(9, -4, d2r(0)),   
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
}
