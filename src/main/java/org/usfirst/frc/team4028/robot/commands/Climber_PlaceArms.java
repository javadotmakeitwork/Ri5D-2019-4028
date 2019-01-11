package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;

import edu.wpi.first.wpilibj.command.Command;

public class Climber_PlaceArms extends Command
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    public Climber_PlaceArms()
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
        _climber.moveArmtoStabilizingPos();
    }

    protected boolean isFinished() 
    {
        return false;
    }

}