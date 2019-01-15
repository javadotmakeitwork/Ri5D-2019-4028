package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.util.BeakXboxController.Thumbstick;

import edu.wpi.first.wpilibj.command.Command;

public class Chassis_DriveWithControllers extends Command
{
    private Chassis _chassis = Chassis.getInstance();
    double _turn = 0; //Intitialized to 0 so that if constructor has no turn it will turn
    private Thumbstick _leftThumbstick;
    private Thumbstick _rightThumbstick;

    public Chassis_DriveWithControllers(Thumbstick leftStick, Thumbstick rightStick) 
    {
        setInterruptible(true);
        _leftThumbstick = leftStick;
        _rightThumbstick = rightStick;
	}

    protected void initialize() {}   

    protected void execute() 
    {
        _chassis.arcadeDrive(-_leftThumbstick.getY(), _rightThumbstick.getX());//forward command is left thumbstick y and 
                                                                                //turn command is right thumbstick x
    }

    protected boolean isFinished() 
    {
        return false;
    }

    protected void interrupted() {}


}