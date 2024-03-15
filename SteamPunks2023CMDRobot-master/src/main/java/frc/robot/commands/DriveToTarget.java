package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CameraSubSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Values;

public class DriveToTarget extends SequentialCommandGroup{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  private final CameraSubSystem camera;

  double sideError = 0;
  double forwardMovement = Values.Variables.currentVelocity;

  double distance = 9999f;

  public DriveToTarget(DriveSubsystem subsystemDrive, CameraSubSystem cameraSubSystem, Command sideAlignCommand, Command driveAligCommand) {
    driveSubsystem = subsystemDrive;
    camera = cameraSubSystem;

    addCommands(
        sideAlignCommand,
        driveAligCommand
    );
  }
  /* 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculateError();
    move();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  private void calculateError(){
    sideError = camera.getYaw();
    distance = camera.getDistance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!camera.isThereATarget()){
        return true;
    }

    if(Math.abs(sideError) <= 0.5f && isDistanceReached()){
        return true;
    }

    return false;
  }

  
  private boolean isDistanceReached(){
    if(camera.getDistance() <= Values.Constants.TargetReachedDistance){
        return true;
    }

    return false;
  }

  private void move(){

    forwardMovement = Values.Variables.currentVelocity;

    if(isDistanceReached()){
      forwardMovement = 0f;
    }

    driveSubsystem.driveCartesian(forwardMovement, Values.Variables.currentVelocity * sideError * 0.5f0, 0);
  }*/
}
