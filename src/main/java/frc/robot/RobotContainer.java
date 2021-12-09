// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.operatorinterface.DualShock4Controller;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  private final DualShock4Controller pilotInput = new DualShock4Controller(Constants.PILOT_JOYSTICK_PORT);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final Command manualDriveCommand = new RunCommand(
    () -> driveSubsystem.arcadeDrive(pilotInput.getY(Hand.kLeft), pilotInput.getX(Hand.kRight))
    , driveSubsystem
  );

  private final Command intakeCommand = new RunCommand(() -> intakeSubsystem.intake(), intakeSubsystem);
  private final Command outtakeCommand = new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem);
  private final Command stopIntakeCommand = new RunCommand(() -> intakeSubsystem.stop(), intakeSubsystem);

  private final Command moveArmToHomeCommand = new RunCommand(()-> armSubsystem.moveTo(ArmPosition.Home), armSubsystem);
  private final Command moveArmToScoreCommand = new RunCommand(()-> armSubsystem.moveTo(ArmPosition.Score), armSubsystem);
  private final Command moveArmToFloorCommand = new RunCommand(()-> armSubsystem.moveTo(ArmPosition.Floor), armSubsystem);

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(manualDriveCommand);

    pilotInput.getLeftBumper()
      .whenHeld(intakeCommand)
      .whenReleased(stopIntakeCommand);

    pilotInput.getRightBumper()
      .whenHeld(outtakeCommand)
      .whenReleased(stopIntakeCommand);

    pilotInput.getTriangleButton()
      .whenPressed(moveArmToHomeCommand);

    pilotInput.getCircleButton()
      .whenPressed(moveArmToScoreCommand);

    pilotInput.getXButton()
      .whenPressed(moveArmToFloorCommand);
  }
}
