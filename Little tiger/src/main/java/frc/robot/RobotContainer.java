// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignAndDriveCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();  
  private final ShooterSubsystem shootersubsystem = new ShooterSubsystem();

  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController supportXbox = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                              () -> driverXbox.getLeftY()*-1,
                                                              () -> driverXbox.getLeftX()*-1)
                                                              .withControllerRotationAxis(() -> driverXbox.getRightX()*-1)
                                                              .deadband(0.1)
                                                              .scaleTranslation(1)
                                                              .allianceRelativeControl(false);
Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
() -> MathUtil.applyDeadband(driverXbox.getLeftY()*-1,0.1),
() -> MathUtil.applyDeadband(driverXbox.getLeftX()*-1,0.1),
() -> driverXbox.getRightX()*-1,
() -> driverXbox.getRightY()*-1);

Command driveFielOrientedAngularVelocity = drivebase.driveFieldOrientedCommand(driveAngularVelocity);
  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);

    NamedCommands.registerCommand("AlignToTag", new RunCommand(() -> drivebase.alignToTag(visionSubsystem), drivebase).raceWith(new WaitCommand(5)));
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(driveFielOrientedAngularVelocity);

    //driverXbox.a().whileTrue(drivebase.alignToTag(visionSubsystem));
    driverXbox.a().whileTrue(new AlignAndDriveCommand(visionSubsystem, drivebase, driverXbox));
    driverXbox.y().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.b().onTrue(driveFielOrientedAngularVelocity);
    driverXbox.x().onTrue(driveFieldOrientedDirectAngle);

    shootersubsystem.setDefaultCommand(shootersubsystem.runShootCommand(() -> driverXbox.getRightTriggerAxis()));

    //driverXbox.rightTrigger().whileTrue(shootersubsystem.runShootCommand(() -> driverXbox.getRightTriggerAxis()));


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
