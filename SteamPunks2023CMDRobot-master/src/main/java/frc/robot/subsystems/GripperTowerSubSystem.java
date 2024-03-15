package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Values;
import frc.robot.commands.move;
import edu.wpi.first.wpilibj.DigitalInput;

public class GripperTowerSubSystem extends SubsystemBase{
    
    public  WPI_TalonSRX gripperExtensionTalon = new WPI_TalonSRX(5); 
    public  WPI_TalonSRX armUpDownTalon = new WPI_TalonSRX(6); 

    private  DigitalInput gripperSwitch = new DigitalInput(5);
    private  DigitalInput armSwitch = new DigitalInput(4);

    private Encoder gripperEncoder = new Encoder(0, 1, true);
    private Encoder armEncoder = new Encoder(2, 3);

    private  Double gripperPosition = 0.0;
    private  Double armPosition = 0.0;

    private float pulseMultiplier = 100000;

    //private Double distPerPulseGripper = (double) (31*100000/1084083);//1037864
    private Double distPerPulseGripper = (double) (100*100000/1084083);//1037864
    //private Double distPerPulseArm = (double) (90*100000/370045); //494540
    private Double distPerPulseArm = (double) (90*100000/494540); //494540

    private Boolean isHoming = true; //TO DO: Change to true when arm fixed.

    public void GripperSubsystem(){
        gripperEncoder.setDistancePerPulse(distPerPulseGripper);
        armEncoder.setDistancePerPulse(distPerPulseArm);
        SmartDashboard.putNumber("DistancePerPulseGripper", distPerPulseGripper);
        SmartDashboard.putNumber("DistancePerPulseArm", distPerPulseArm);

        isHoming = true;
    }

    private void home(){

        if(gripperSwitch.get()){ //Gripper not homed
            moveGripper(-0.5f);
        }else{ //Gripper homed
            moveGripper(0);
            if(!armSwitch.get()){ //Arm not homed
                moveArm(-0.5f);
            }else{
                moveArm(0);
                isHoming = false;
                armEncoder.reset();
                gripperEncoder.reset();
                gripperEncoder.setDistancePerPulse(distPerPulseGripper);
                armEncoder.setDistancePerPulse(distPerPulseArm);
            }
        }
    }

    private void readEncodersPosition(){
        gripperPosition = gripperEncoder.getDistance()/100000.0;
        armPosition = armEncoder.getDistance()/100000.0;

        SmartDashboard.putNumber("Gripper Encoder Ticks", gripperEncoder.get());
		SmartDashboard.putNumber("Gripper Distance", gripperPosition);
        SmartDashboard.putNumber("Arm Encoder Ticks", armEncoder.get());
		SmartDashboard.putNumber("Arm Distance", armPosition);
        SmartDashboard.putNumber("armPercent", armUpDownTalon.getMotorOutputPercent()*100);
    }

    @Override
    public void periodic(){

         if(isHoming){
            home();
            armEncoder.reset();
            gripperEncoder.reset();
            gripperEncoder.setDistancePerPulse(distPerPulseGripper);
            armEncoder.setDistancePerPulse(distPerPulseArm);
        } 
        
        readEncodersPosition();

        SmartDashboard.putBoolean("gripperSwitch", gripperSwitch.get());
		SmartDashboard.putBoolean("armSwitch", armSwitch.get());
        SmartDashboard.putNumber("armCurrent", armUpDownTalon.getSupplyCurrent());
        SmartDashboard.putBoolean("armOverVoltage", armUpDownTalon.getMotorOutputPercent() > Values.Constants.armPercentLimit);
        SmartDashboard.putNumber("gripperCurrent", gripperExtensionTalon.getSupplyCurrent());
        SmartDashboard.putBoolean("gripperOverVoltage", gripperExtensionTalon.getMotorOutputPercent() > Values.Constants.gripperPercentLimit);
        SmartDashboard.putNumber("armPercent", armUpDownTalon.getMotorOutputPercent()*100);
        SmartDashboard.putNumber("gripperPercent", gripperExtensionTalon.getMotorOutputPercent()*100);
    }


    public double getArmPosition(){
        return armPosition;
    }

    public double getGripperPosition(){
        return gripperPosition;
    }

    public void moveArm(float speed){

        if(armSwitch.get() && speed < 0){ //arm homed
            armUpDownTalon.set(ControlMode.PercentOutput, 0);

        }else{
            armUpDownTalon.set(ControlMode.PercentOutput, speed);
        }
    }

    public void moveGripper(float speed){
        
        if(!gripperSwitch.get() && speed <= 0){ //Gripper homed
            gripperExtensionTalon.set(ControlMode.PercentOutput, 0);

        }else{
            gripperExtensionTalon.set(ControlMode.PercentOutput, speed);
        }
    }

    public void stop(){
        gripperExtensionTalon.set(ControlMode.PercentOutput, 0);
        armUpDownTalon.set(ControlMode.PercentOutput, 0);
    }

    public Boolean isGripperHomed(){
        return !gripperSwitch.get();
    }

    public Boolean isArmHomed(){
        return armSwitch.get();
    }

    public void resetGripperEncoder(){
        gripperEncoder.reset();
        gripperEncoder.setDistancePerPulse(distPerPulseGripper);
    }

    public void resetArmEncoder(){
        armEncoder.reset();
        armEncoder.setDistancePerPulse(distPerPulseArm);
    }
}
