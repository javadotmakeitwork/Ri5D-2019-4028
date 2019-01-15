package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;
import org.usfirst.frc.team4028.robot.util.BeakXboxController.Trigger;

import edu.wpi.first.wpilibj.command.Command;

public class Climber_PlaceArms extends Command
{
    Trigger _rightTrigger;
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    public Climber_PlaceArms(Trigger rightTrigger)//places arms on the front of the robot in order to stabilize the robot while climbing
    {
        setInterruptible(true);
        _rightTrigger = rightTrigger;
    }
    @Override
    protected void initialize() 
    {
        
    }

    @Override
    protected void execute() 
    {
        _climber.moveArmtoStabilizingPos(_rightTrigger.getX());//send the arms out on the command of a trigger
    }

    protected boolean isFinished() 
    {
        return false;
    }

}