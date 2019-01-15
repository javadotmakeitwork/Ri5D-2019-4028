/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4028.robot;

// #region Import Statements
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;


/**
 * The VM is configured to automatically run this class
 */
public class Robot extends TimedRobot 
{
	
	// create instance of singelton Subsystems
		
	private Chassis _chassis = Chassis.getInstance();
	private ElevatorClimber _climber = ElevatorClimber.getInstance();
	private OI _oi = OI.getInstance();
 	
	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() 
	{
		_chassis.stop();

		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() 
	{
		Scheduler.getInstance().run();

	}

	/**
	 * This method runs 1x when the robot enters auton mode
	 */
	@Override
	public void autonomousInit() 
	{
		_chassis.stop();
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() 
	{
		Scheduler.getInstance().run();		
	}

	/**
	 * This method runs 1x when the robot enters telop mode
	 */
	@Override
	public void teleopInit() 
	{
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		_chassis.stop();
		_climber.moveElevator(0);
		_climber.setElevatorPos(0);
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() 
	{
		Scheduler.getInstance().run();

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {}

}
