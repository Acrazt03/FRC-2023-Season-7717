// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Values {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
  }
  public static final class Constants {
    
    //CAMERA VARIABLES
    public static final float CAMERAHMETERS = 0.61f;
    public static final float TARGETHMETERS = 0.53f;
    public static final float CAMERAPITCH = 0f;

    public static final float TargetReachedDistance = 2f;

    //DRIVETRAIN VARIABLES\\
    public static final float initialVelocity = 0.7f;
    public static final float sideVelocity = 0.3f;
    public static final float turboVelocity = 0.7f;

    public static final float rotationSpeed = 0.7f;

    public static final float extendGripperSpeed = 0.80f;
    public static final float extendArmSpeed = 1f;

    //TALON IDs
    public static final int  talonBackRightID = 3;
    public static final int  talonBackLeftID = 2;
    public static final int  talonFrontRightID = 4;
    public static final int  talonFrontLeftID =  1;

    //GRIPPER TALON IDs
    public static final int gripperExtansionTalonID = 6;
    public static final int gripperUpDownTalonID = 5;

    //vOLTAGE REgulator variaables
    public static final float armPercentLimit = 0.5f;
    public static final float gripperPercentLimit = 0.5f;
    public static final float drivetrainPercentLimit = 0.5f;
  }

  public static final class Variables {
    public static float currentVelocity = Constants.initialVelocity;
    public static float currentSideSpeed = Constants.initialVelocity; //different speed to make side movement faster or slower for mecanum.
    public static Boolean isInMecanumMode = false;
    public static Boolean previousMecanumMode = !isInMecanumMode;
  }
}



