// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final PWMSparkMax motor = new PWMSparkMax(Constants.MOTOR_INTAKE);

  public IntakeSubsystem() {}

  public void start(String armPosition){
    if (armPosition == "up") {
      motor.set(0.5);
    } 
    else if (armPosition == "down") {
      motor.set(-0.5);
    }
  }

  public void stop(){
    motor.set(0);
  }
}
