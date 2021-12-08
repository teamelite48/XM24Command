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

  private double armOffset = ArmPosition.Home.degrees;

  private final DigitalInput upperLimitSwitch = new DigitalInput(Constants.ARM_UP_LIMIT_SWITCH_CHANNEL);
  private final DigitalInput lowerLimitSwith = new DigitalInput(Constants.ARM_DOWN_LIMIT_SWITCH_CHANNEL);

  public ArmSubsystem() {}

  public boolean isArmInClimablePosition() {
    return isArmAtUpperLimit() || isArmAtLowerLimit();
  }

  public void moveToHomePosition() {
    moveToPosition(ArmPosition.Home);
  }

  public void moveToScorePosition() {
    moveToPosition(ArmPosition.Score);
  }

  public void moveToFloorPosition() {
    moveToPosition(ArmPosition.Floor);
  }

  private void moveToPosition(ArmPosition desiredArmPosition) {

    if (isArmTryingToPassLimit(desiredArmPosition)) {
      stopMotor();
      homeArmPosition();
      return;
    }

    double currentDegrees = getArmPositionInDegrees();
    double desiredDegrees = desiredArmPosition.degrees;

    double speed = pid.calculate(currentDegrees, desiredDegrees);

    motor.set(speed);
  }

  private double getArmPositionInDegrees() {
    return armOffset + ((encoder.getPosition() / Constants.ARM_ENCODER_TICKS_PER_REV) * Constants.ARM_ENCODER_DISTANCE_PER_REV);
  }

  private boolean isArmTryingToPassLimit(ArmPosition desiredArmPosition) {

    if (isArmAtUpperLimit() && desiredArmPosition == ArmPosition.Home) {
      return true;
    }

    if (isArmAtLowerLimit() && desiredArmPosition == ArmPosition.Floor) {
      return true;
    }

    return false;
  }

  private boolean isArmAtUpperLimit() {
    return !upperLimitSwitch.get();
  }

  private boolean isArmAtLowerLimit() {
    return !lowerLimitSwith.get();
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
      armOffset = ArmPosition.Home.degrees;
    }

    else if (isArmAtLowerLimit()) {
      armOffset = ArmPosition.Floor.degrees;
    }
  }

  private void resetEncoder() {
    encoder.setPosition(0.0);
  }
}
