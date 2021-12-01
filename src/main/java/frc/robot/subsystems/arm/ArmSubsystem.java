// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(Constants.ARM_MOTOR_DEVICE_ID, CANSparkMax.MotorType.kBrushless);
  private final CANEncoder encoder = motor.getEncoder();

  private  final PIDController pid = new PIDController(Constants.ARM_KP, Constants.ARM_KI, Constants.ARM_KD);

  private double armOffset = Constants.ARM_HOME_DEGREES;

  private final DigitalInput upLimit = new DigitalInput(Constants.ARM_UP_LIMIT_SWITCH_CHANNEL);
  private final DigitalInput downLimit = new DigitalInput(Constants.ARM_DOWN_LIMIT_SWITCH_CHANNEL);

  public ArmSubsystem() {}

  public void moveToHomePosition() {
    setArmAngle(Constants.ARM_HOME_DEGREES);
  }

  public void moveToScorePosition() {
    setArmAngle(Constants.ARM_SCORE_DEGREES);
  }

  public void moveToFloorPosition() {
    setArmAngle(Constants.ARM_FLOOR_DEGRESS);
  }

  public double getArmAngle() {
    return armOffset + ( (encoder.getPosition() / Constants.ARM_ENCODER_TICKS_PER_REV) * Constants.ARM_ENCODER_DISTANCE_PER_REV);
  }

  public void setArmAngle(double desiredAngle) {

    if (isArmTryingToPassLimit(desiredAngle)) {
      stopMotor();
      homeArmPosition();
      return;
    }

    double currentAngle = getArmAngle();
    double speed = pid.calculate(currentAngle, desiredAngle);

    motor.set(speed);
  }

  public boolean isArmHome() {
    return isArmAtUpperLimit() || isArmAtLowerLimit();
  }

  private boolean isArmTryingToPassLimit(double desiredAngle) {
    
    if (isArmAtUpperLimit() && desiredAngle == Constants.ARM_HOME_DEGREES) {
      return true;
    }

    if (isArmAtLowerLimit() && desiredAngle == Constants.ARM_FLOOR_DEGRESS) {
      return true;
    }

    return false;
  }

  private boolean isArmAtUpperLimit() {
    return !upLimit.get();
  }

  private boolean isArmAtLowerLimit() {
    return !downLimit.get();
  }

  private void stopMotor() {
    motor.set(0);
  }

  private void homeArmPosition() {
    updateArmOffset();
    resetEncoder();
  }

  private void updateArmOffset() {

    if (isArmAtUpperLimit()) {
      armOffset = Constants.ARM_HOME_DEGREES;
    }

    else if (isArmAtLowerLimit()) {
      armOffset = Constants.ARM_FLOOR_DEGRESS;
    }
  }

  private void resetEncoder() {
    encoder.setPosition(0.0);
  }
}
