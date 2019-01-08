package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;

import edu.wpi.first.wpilibj.command.Command;

public class Climber_GetDownFromLvl2 extends Command
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    public Climber_GetDownFromLvl2()
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
        _climber.driveDown();
    }
    @Override
    protected boolean isFinished() 
    {
        return false;
    }
}