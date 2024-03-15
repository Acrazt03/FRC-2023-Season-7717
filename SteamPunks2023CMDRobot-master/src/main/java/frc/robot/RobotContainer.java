// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveToTarget;
import frc.robot.commands.HomeArm;
import frc.robot.commands.HomeGripper;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveGripper;
import frc.robot.commands.MoveTower;
import frc.robot.commands.move;
import frc.robot.subsystems.CameraSubSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GearSubSystem;
import frc.robot.subsystems.GripperTowerSubSystem;
import frc.robot.subsystems.HandSubSystem;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.Commands;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final CameraSubSystem cameraSubSystem = new CameraSubSystem();
  public final GearSubSystem gearSubSystem = new GearSubSystem();

  public final GripperTowerSubSystem gripperTowerSubSystem = new GripperTowerSubSystem();

  public final HandSubSystem handSubSystem = new HandSubSystem();

  private Command m_driveCommand;
  private Command m_driveToTargeTCommand;

  private Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    initializeHardware();
    configureBindings();
  }

  private void initializeHardware(){
    pcmCompressor.enableDigital();
  }

  private void configureBindings() {
    configureTelopBindings();
  }

  public void configureTelopBindings(){
    
    Trigger placeObjectTrigger = Inputs.commandController.a();

    Trigger moveTrigger = Inputs.commandController.rightTrigger(0.5f);

    Trigger openGripper = Inputs.commandController.y();
    //Trigger testTowerMov = Inputs.commandController.b();
    //Trigger moveToTargetTrigget = Inputs.commandController.a();
     
    Trigger changeTransmissionTrigger = Inputs.commandController.rightBumper();

    m_driveToTargeTCommand = getDriveToTargetCommand();
    //moveToTargetTrigget.onTrue(m_driveToTargeTCommand);
    
    moveTrigger.whileTrue(getDriveCommand());

    /*testTowerMov.onTrue(
      new MoveTower(
        gripperTowerSubSystem,
        45.0, 20.0, false));*/

    
    openGripper.onTrue(
      new RunCommand(
        () -> handSubSystem.openGripper(),
        handSubSystem)
    ).onFalse(
        new RunCommand(
          () -> handSubSystem.closeGripper(),
          handSubSystem)
    );

   /* changeTransmissionTrigger.onTrue(
      new RunCommand(
        () -> gearSubSystem.ChangeToTranctionWheels(),
        gearSubSystem)
    ).onFalse(
        new RunCommand(
          () -> gearSubSystem.ChangeToMecanum(),
          gearSubSystem)
    );*/
        changeTransmissionTrigger.onTrue(
      new RunCommand(
        () -> gearSubSystem.changeWheels(),
        gearSubSystem)
    ).onFalse(
        changeBooleanCommand()
    );
  

    /*Trigger homeGripper = Inputs.commandController.povLeft();
    Trigger moveGripper = Inputs.commandController.povRight();
    Trigger moveArm = Inputs.commandController.povUp();
    Trigger homeArm = Inputs.commandController.povDown();

    homeGripper.onTrue(new HomeGripper(gripperTowerSubSystem));
    homeArm.onTrue(new HomeArm(gripperTowerSubSystem));
    moveGripper.onTrue(new MoveGripper(gripperTowerSubSystem, 50.0));
    moveArm.onTrue(new MoveArm(gripperTowerSubSystem, 60.0));
    */
    Trigger extendGripperTrigger = Inputs.commandController.povLeft();
    Trigger detractGripperTrigger = Inputs.commandController.povRight();
    Trigger raiseArmTrigger = Inputs.commandController.povUp();
    Trigger lowerArmTrigger = Inputs.commandController.povDown();

  /*   raiseArmTrigger.whileTrue(
      Commands.startEnd(
           // Starts arm movement
           () ->gripperTowerSubSystem.moveArm(0.5f),
           // Stops arm movement
           () ->gripperTowerSubSystem.moveArm(0.0f),
          gripperTowerSubSystem
      )
    );*/

    raiseArmTrigger.whileTrue(
      new FunctionalCommand(
      () -> doNothing(),
      () -> gripperTowerSubSystem.moveArm(0.5f),
      interrupted -> gripperTowerSubSystem.moveArm(0.0f),
      () -> gripperTowerSubSystem.armUpDownTalon.getMotorOutputPercent()>Values.Constants.armPercentLimit,
      // Require the drive subsystem
      gripperTowerSubSystem
      )
    );

    lowerArmTrigger.whileTrue(
      new FunctionalCommand(
        () -> doNothing(),
        () -> gripperTowerSubSystem.moveArm(-0.5f),
        interrupted -> gripperTowerSubSystem.moveArm(0.0f),
        () -> Math.abs(gripperTowerSubSystem.armUpDownTalon.getMotorOutputPercent())>Values.Constants.armPercentLimit,
        // Require the drive subsystem
        gripperTowerSubSystem
        )
    );

    extendGripperTrigger.whileTrue(
      new FunctionalCommand(
      () -> doNothing(),
      () -> gripperTowerSubSystem.moveGripper(0.5f),
      interrupted -> gripperTowerSubSystem.moveGripper(0.0f),
      () -> gripperTowerSubSystem.gripperExtensionTalon.getMotorOutputPercent()>Values.Constants.gripperPercentLimit,
      // Require the drive subsystem
      gripperTowerSubSystem
      )
    );

    detractGripperTrigger.whileTrue(
      new FunctionalCommand(
        () -> doNothing(),
        () -> gripperTowerSubSystem.moveGripper(-0.5f),
        interrupted -> gripperTowerSubSystem.moveGripper(0.0f),
        () -> Math.abs(gripperTowerSubSystem.gripperExtensionTalon.getMotorOutputPercent())>Values.Constants.gripperPercentLimit,
        // Require the drive subsystem
        gripperTowerSubSystem
        )
    );

    placeObjectTrigger.onTrue(placeObjectAutonomousCommand());
  }

  public void doNothing(){

  }

  public FunctionalCommand getSideAlignCommand(){
    return new FunctionalCommand(
      () -> cameraSubSystem.getYaw(),
      () -> moveSide(),
      interrupted -> driveSubsystem.driveCartesian(0,0, 0),
      // End the command when the robot's driven distance is bellow the desired value
      () -> Math.abs(cameraSubSystem.getYaw()) <= 0.5f,
      // Require the drive subsystem
      driveSubsystem,cameraSubSystem
      );
  }

  public FunctionalCommand getDriveAlignCommand(){
    return new FunctionalCommand(
      () -> resetTest(),
      () -> moveForward(),
      interrupted -> driveSubsystem.driveCartesian(0,0, 0),
      // End the command when the robot's driven distance is bellow the desired value
      () -> cameraSubSystem.getDistance() <= 0,
      // Require the drive subsystem
      driveSubsystem,cameraSubSystem
      );
  }

  private void resetTest(){

  }

  private void moveSide(){
    double sideError = cameraSubSystem.getYaw();
    driveSubsystem.driveCartesian(0, sideError * Values.Constants.sideVelocity, 0);
  }
  private void moveForward(){
    driveSubsystem.driveCartesian(Values.Variables.currentVelocity, 0, 0);
  }

  /*public static CommandBase placeObjectCommand(){
    new Commands.sequence(
      new MoveArm(gripperTowerSubSystem, 70.0),
      new MoveGripper(gripperTowerSubSystem, 15.0),
      new RunCommand(
        () -> handSubSystem.openGripper(),
        handSubSystem)
    );
  }*/

  public Command placeObjectCommand(){
    
    Command moveArmCMD = new MoveArm(gripperTowerSubSystem, 80.0);
    Command moveGripperCMD = new MoveGripper(gripperTowerSubSystem, 77.0);
    Command openGripperCMD = new RunCommand(
      () -> handSubSystem.openGripper(),
      handSubSystem);
    
    return moveArmCMD.andThen(moveGripperCMD).andThen(Commands.waitSeconds(0.5)).andThen(openGripperCMD);
  }

  public Command homeAll(){
    Command homeGripper = new HomeGripper(gripperTowerSubSystem);
    Command closeGripperCMD = new RunCommand(
      () -> handSubSystem.closeGripper(),
      handSubSystem);
    Command homeArm = new HomeArm(gripperTowerSubSystem);
  
  return homeGripper.andThen(closeGripperCMD).andThen(homeArm);
  }

  public Command placeObjectAutonomousCommand(){
    Command placeObjectCMD = placeObjectCommand();
    Command homeAllCMD = homeAll();

    return placeObjectCMD.andThen(homeAllCMD);
  }

  public Command getAutonomousCommand() {
    return getDriveToTargetCommand();
  }

  public Command getDriveToTargetCommand() {
    // An example command will be run in autonomous
    
    Command command = new DriveToTarget(driveSubsystem, cameraSubSystem, getSideAlignCommand(), getDriveAlignCommand());

    return command;
    //return Autos.exampleAuto(m_exampleSubsystem);
  }

  public Command getDriveCommand(){
    Command command = new move(driveSubsystem);

    return command;
  }

  public Command changeBooleanCommand(){
    return new FunctionalCommand(
      () -> Values.Variables.previousMecanumMode = Values.Variables.isInMecanumMode,
      () -> gearSubSystem.changeBoolean(),
      interrupted -> doNothing(),
      () -> Values.Variables.isInMecanumMode == !Values.Variables.previousMecanumMode,
      // Require the drive subsystem
      gearSubSystem
      );
    
  }
}
