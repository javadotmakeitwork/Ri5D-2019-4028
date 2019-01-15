package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;

import edu.wpi.first.wpilibj.command.Command;

public class Climber_StopArms extends Command
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    public Climber_StopArms()
    {

    }
    @Override
    protected void initialize() 
    {
        _climber.stopArms();//stops the arms once the trigger is released
    }

    @Override
    protected boolean isFinished() 
    {
        return true;
    }
}