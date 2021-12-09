// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
 
  private final TalonFX motor = new TalonFX(Constants.CLIMBER_MOTOR_DEVICE_ID);

  public ClimberSubsystem() {
    motor.setSelectedSensorPosition(0, 0, Constants.TIMEOUT_MS);
  }

  @Override
  public void periodic() {}
}
