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
        _climber.keepElevatorAtCurrentHeight();//the elevator needs to be told to stay at the target position only once as the 
                                                //command will not change until a new one is given
    }
    @Override
    protected boolean isFinished() 
    {
        return true;//only needs to run once
    }
}