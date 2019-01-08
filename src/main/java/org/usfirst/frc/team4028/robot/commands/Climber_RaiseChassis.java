package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;

import edu.wpi.first.wpilibj.command.Command;

public class Climber_RaiseChassis extends Command
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    public Climber_RaiseChassis()
    {
        setInterruptible(true);
    }
    @Override
    protected void initialize() 
    {
        
    }
    @Override
    protected void execute() 
    {
        _climber.elevate();
    }
    @Override
    protected boolean isFinished() 
    {
        return _climber.isRobotElevated();
    }

}