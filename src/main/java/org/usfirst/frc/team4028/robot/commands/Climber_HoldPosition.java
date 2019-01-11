package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;

import edu.wpi.first.wpilibj.command.Command;

public class Climber_HoldPosition extends Command
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    public Climber_HoldPosition()
    {
        setInterruptible(false);
    }
    @Override
    protected void initialize() 
    {
        _climber.keepElevatorAtCurrentHeight();
    }
    @Override
    protected boolean isFinished() 
    {
        return true;
    }
}