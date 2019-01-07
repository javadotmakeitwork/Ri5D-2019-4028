package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;

import edu.wpi.first.wpilibj.command.Command;

class Climber_RestoreElevator extends Command
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    public Climber_RestoreElevator()
    {
        setInterruptible(false);
    }
    @Override
    protected void initialize() 
    {
        
    }
    @Override
    protected void execute() 
    {
        _climber.zeroElevator();
    }
    @Override
    protected boolean isFinished() 
    {
        return _climber.hasElevatorBeenZeroed();
    }

}
