// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private ArmPosition armPosition = ArmPosition.Home;

  public ArmSubsystem() {}

  public void moveToHomePosition() {
    armPosition = ArmPosition.Home;
  }

  public void moveToScorePosition() {
    armPosition = ArmPosition.Score;
  }

  public void moveToFloorPosition() {
    armPosition = ArmPosition.Floor;
  }

  public ArmPosition getArmPosition() {
    return armPosition;
  }
}
