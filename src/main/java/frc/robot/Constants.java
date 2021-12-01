// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
  public static final int MOTOR_LEFT_FRONT = 3;
  public static final int MOTOR_LEFT_REAR = 2;
  public static final int MOTOR_RIGHT_FRONT = 0;
  public static final int MOTOR_RIGHT_REAR = 1;
  
  public static final int MOTOR_INTAKE = 5;

  public static final int ENCODER_LEFT_A = 0;
  public static final int ENCODER_LEFT_B = 1;
  public static final int ENCODER_RIGHT_A = 2;
  public static final int ENCODER_RIGHT_B = 3;

  public static final double SIMULATION_REFRESH_RATE = 0.02;
  public static final int ENCODER_RESOLUTION = -4096;
  public static final int GYRO_CHANNEL = 0;

  public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
  
  public static final int PILOT_JOYSTICK_PORT = 0;

  public static final int ARM_MOTOR_DEVICE_ID = 10;

  public static final double ARM_KP =	0.026;    				
  public static final double ARM_KI =	0.0003;					
  public static final double ARM_KD =  0.005;	
  
  public static final double ARM_HOME_DEGREES = 0.0;	
  public static final double ARM_SCORE_DEGREES = 35.0;
  public static final double ARM_FLOOR_DEGRESS = 90.0;

   public static final double ARM_ENCODER_TICKS_PER_REV = 42.0/48.76;
   public static final double ARM_ENCODER_DISTANCE_PER_REV = 3.0;   
   
   public static final int ARM_DOWN_LIMIT_SWITCH_CHANNEL = 4;
	 public static final int ARM_UP_LIMIT_SWITCH_CHANNEL = 5;	 	 
}
