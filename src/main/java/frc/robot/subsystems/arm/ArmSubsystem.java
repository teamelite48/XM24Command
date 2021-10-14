// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(Constants.ARM_MOTOR_DEVICE_ID, CANSparkMax.MotorType.kBrushless);
  private final CANEncoder encoder = motor.getEncoder();

  private  final PIDController pid = new PIDController(Constants.ARM_KP, Constants.ARM_KI, Constants.ARM_KD);

  private final int armOffset = 0;

  public ArmSubsystem() {}

  public void moveToHomePosition() {
    setArmAngle(Constants.ARM_HOME);
  }

  public void moveToScorePosition() {
    setArmAngle(Constants.ARM_SCORE);
  }

  public void moveToFloorPosition() {
    setArmAngle(Constants.ARM_FLOOR);
  }

  public double getArmAngle() {
    return armOffset + ( (encoder.getPosition() / Constants.ARM_ENCODER_TICKS_PER_REV) * Constants.ARM_ENCODER_DISTANCE_PER_REV);
  }

  public void setArmAngle(double angle) {
    double pidOutput = pid.calculate(getArmAngle(), angle);
    motor.set(pidOutput);
  }
}
