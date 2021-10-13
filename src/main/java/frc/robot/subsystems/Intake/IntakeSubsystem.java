// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmPosition;

public class IntakeSubsystem extends SubsystemBase {

  private final PWMSparkMax motor = new PWMSparkMax(Constants.MOTOR_INTAKE);

  public IntakeSubsystem() {}

  public void start(ArmPosition armPosition){
    switch (armPosition) {
      case Up: {
        outtake();
        break;
      }
      case Down: {
        intake();
        break;
      }
    }
  }

  public void stop(){
    motor.set(0);
  }

  private void intake() {
    motor.set(0.5);
  }

  private void outtake() {
    motor.set(-0.5);
  }
}
