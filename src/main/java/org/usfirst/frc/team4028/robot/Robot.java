/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4028.robot;

// #region Import Statements
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.util.Date;

import org.usfirst.frc.team4028.robot.auton.Paths;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;


/**
 * The VM is configured to automatically run this class
 */
public class Robot extends TimedRobot 
{
	
	// create instance of singelton Subsystems
	private Dashboard _dashboard = Dashboard.getInstance();
	
	private Chassis _chassis = Chassis.getInstance();
	private OI _oi = OI.getInstance();



	// class level working variables
 	long _lastDashboardWriteTimeMSec;

 	
	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() 
	{
		_chassis.stop();
		Paths.buildPaths();

		
		outputAllToDashboard();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();

	}

	/**
	 * This method runs 1x when the robot enters auton mode
	 */
	@Override
	public void autonomousInit() {
		_chassis.stop();
		_dashboard.getSelectedAuton().start();
		Scheduler.getInstance().run();


		_chassis.setHighGear(true);

		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
		_dashboard.outputToDashboard();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() 
	{

		Scheduler.getInstance().run();
		_chassis.updateChassis(Timer.getFPGATimestamp());
		// System.out.println(_chassis.isDoneWithPath());
		
		// ============= Refresh Dashboard =============
		_dashboard.outputToDashboard();
		outputAllToDashboard();
		
		// ============= Optionally Log Data =============
		
	}

	/**
	 * This method runs 1x when the robot enters telop mode
	 */
	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		_chassis.stop();
		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		// ============= Refresh Dashboard =============
		outputAllToDashboard();
		_dashboard.outputToDashboard();
		
		// ============= Optionally Log Data =============

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {}
	
	/** Method to Push Data to ShuffleBoard */
	private void outputAllToDashboard() {
		// limit spamming
    	// add scan time sample to calc scan time rolling average
    	//_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
    		// each subsystem should add a call to a outputToSmartDashboard method
    		// to push its data out to the dashboard

    		_chassis.updateDashboard(); 

	    	
    		// write the overall robot dashboard info
	    	
	    	//BigDecimal movingAvg = _scanTimeSamples.getAverage();
	    	//DecimalFormat df = new DecimalFormat("####");
	    	//SmartDashboard.putString("Scan Time (2 sec roll avg)", df.format(movingAvg) + " mSec");
    		// snapshot last time
    		_lastDashboardWriteTimeMSec = new Date().getTime();
    	}
    	
    	// snapshot when this scan ended
	}
	

}
