// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private ArmPosition armPosition = ArmPosition.Up;

  public ArmSubsystem() {}

  public void toggleArm () {
    switch (armPosition) {
      case Up: {
        armPosition = ArmPosition.Down;
        break;
      }
      case Down: {
        armPosition = ArmPosition.Up;
        break;
      }
    }
  }

  public ArmPosition getArmPosition() {
    return armPosition;
  }
}
