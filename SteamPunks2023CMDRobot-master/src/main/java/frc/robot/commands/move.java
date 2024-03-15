package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Values;
import frc.robot.Inputs;

public class move extends CommandBase {
    private final DriveSubsystem driveSubsystem; 

    public move(DriveSubsystem subsystem){
        driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double rightJoystick = Inputs.commandController.getRightX();

        double xMove = Inputs.commandController.getLeftY();
        double yMove = Inputs.commandController.getLeftX();
        float kRot = 0;

        if(rightJoystick < -0.5f){
            kRot = 1f;
        }

        if(rightJoystick > 0.5f){
            kRot = -1f;
        }

        driveSubsystem.driveCartesian(Values.Variables.currentSideSpeed * xMove, Values.Variables.currentVelocity * yMove, Values.Constants.rotationSpeed *kRot);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
