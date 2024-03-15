// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  MotorController frontLeftMotor = new WPI_TalonSRX(1);
  MotorController backLeftMotor = new WPI_TalonSRX(2);
  MotorController frontRightMotor = new WPI_TalonSRX(4);
  MotorController backRightMotor = new WPI_TalonSRX(3);

  /*MotorControllerGroup rightGroup = new MotorControllerGroup(frontRightMotor,backRightMotor);
  MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeftMotor,backLeftMotor);

  DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);
  */
  
  MecanumDrive mecanumDrive =  new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  WPI_TalonSRX gripperExtansionTalon = new WPI_TalonSRX(5); 
  WPI_TalonSRX gripperUpDownTalon = new WPI_TalonSRX(6); 

  public XboxController xboxController = new XboxController(0);
  private Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
   
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  DigitalInput gripperSwitch = new DigitalInput(5);
  DigitalInput armSwitch = new DigitalInput(4);
  private Encoder gripperEncoder = new Encoder(0, 1);
  private Encoder armEncoder = new Encoder(2, 3);
 

  //boolean enabled = pcmCompressor.enabled();
  //boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
  //double current = pcmCompressor.getCompressorCurrent();

  //Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  DoubleSolenoid transmissioSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid gripperSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  //private Double distPerPulseGripper = (double) (31*100000/1084083);//1037864
  private Double distPerPulseGripper = (double) (100*100000/1084083);//1037864
 
  //private Double distPerPulseArm = (double) (90*100000/370045); //494540
  private Double distPerPulseArm = (double) (90*100000/494540); //494540

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    gripperEncoder.reset();
    armEncoder.reset();

    pcmCompressor.enableDigital();

    transmissioSolenoid.set(kOff);
    gripperSolenoid.set(kOff);

    mecanumDrive.setMaxOutput(0.75f);

    gripperEncoder.setDistancePerPulse(distPerPulseGripper);
    armEncoder.setDistancePerPulse(distPerPulseArm);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("DistancePerPulseGripper", distPerPulseGripper);
    SmartDashboard.putNumber("DistancePerPulseArm", distPerPulseArm);

    var gripperPosition = gripperEncoder.getDistance()/100000.0;
    var armPosition = armEncoder.getDistance()/100000.0;

    SmartDashboard.putNumber("Gripper Encoder Ticks", gripperEncoder.get());
    SmartDashboard.putNumber("Gripper Distance", gripperPosition);
    SmartDashboard.putNumber("Arm Encoder Ticks", armEncoder.get());
    SmartDashboard.putNumber("Arm Distance", armPosition);

    SmartDashboard.putBoolean("gripperSwitch", gripperSwitch.get());
		SmartDashboard.putBoolean("armSwitch", armSwitch.get());

    mecanumDrive.driveCartesian(-xboxController.getRightX(), -xboxController.getLeftX(), xboxController.getLeftY());

    int pov = xboxController.getPOV();
    SmartDashboard.putNumber("POV", pov);

    switch(pov){
      case 0:
        gripperUpDownTalon.set(ControlMode.PercentOutput, 0.75f);
        break;
      case 180:
        gripperUpDownTalon.set(ControlMode.PercentOutput, -0.75f);
        break;
      case 90:
      gripperExtansionTalon.set(ControlMode.PercentOutput, 1);
        break;
      case 270:
      gripperExtansionTalon.set(ControlMode.PercentOutput, -1);
        break;
      default:
        gripperExtansionTalon.set(ControlMode.PercentOutput, 0f);
        gripperUpDownTalon.set(ControlMode.PercentOutput, 0f);
    }

    if(xboxController.getRightBumper()){
      transmissioSolenoid.set(kForward);
    }else if(xboxController.getLeftBumper()){
      transmissioSolenoid.set(kReverse);
    }

    if(xboxController.getBButton()){
      gripperSolenoid.set(kForward);

    }else if(xboxController.getXButton()){
      gripperSolenoid.set(kReverse);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
