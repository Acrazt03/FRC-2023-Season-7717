package frc.robot.subsystems;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Values;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  public WPI_TalonSRX talonBackRight = new WPI_TalonSRX(Values.Constants.talonBackRightID);
  public WPI_TalonSRX talonBackLeft = new WPI_TalonSRX(Values.Constants.talonBackLeftID);
  public WPI_TalonSRX talonFrontRight = new WPI_TalonSRX(Values.Constants.talonFrontRightID);
  public WPI_TalonSRX talonFrontLeft = new WPI_TalonSRX(Values.Constants.talonFrontLeftID);
  
  //private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(talonBackLeft,talonFrontLeft);

  // The motors on the right side of the drive.
  //private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(talonBackRight, talonFrontRight);

  // The robot's drive
  //private final DifferentialDrive m_diff_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  private final MecanumDrive m_mecanum_drive = new MecanumDrive(talonFrontLeft, talonBackLeft, talonFrontRight, talonBackRight);

  public DriveSubsystem() {

    talonBackLeft.setInverted(true);
    talonFrontLeft.setInverted(true);
    //m_rightMotors.setInverted(true);
    setMaxOutput(0.75f);
  } 

  @Override
  public void periodic() {}

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  /*public void arcadeDrive(double fwd, double rot) {
    m_diff_drive.arcadeDrive(fwd, rot);
  }*/

  /*public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diff_drive.tankDrive(leftSpeed, rightSpeed);
  }*/
  
  public void driveCartesian(double xSpeed, double ySpeed, double zRotation){
    m_mecanum_drive.driveCartesian(xSpeed, -ySpeed, zRotation);
    //m_mecanum_drive.drivePolar(ySpeed, null, zRotation);
  
    //talonFrontLeft.set(TalonSRXControlMode.PercentOutput, xSpeed);
    //talonBackRight.set(TalonSRXControlMode.PercentOutput, ySpeed);
    //talonFrontLeft.set(TalonSRXControlMode.PercentOutput, zRotation);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  /*public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_diff_drive.feed();
  }*/
  public void setMaxOutput(double maxOutput) {
    //m_diff_drive.setMaxOutput(maxOutput);
    m_mecanum_drive.setMaxOutput(maxOutput);
  }
}