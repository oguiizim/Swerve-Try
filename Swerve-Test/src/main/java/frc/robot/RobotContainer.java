// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controle;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

  public static final Swerve swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
  public static final XboxController control = new XboxController(0);
  public static final CommandXboxController driver = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    swerve.setDefaultCommand(new DriveCommand(swerve,
        () -> -MathUtil.applyDeadband(driver.getLeftX(), Controle.DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getLeftY(), Controle.DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getRawAxis(2), Controle.DEADBAND),
        control));

    Command driveSim = swerve.simDriveCommand(() -> MathUtil.applyDeadband(driver.getLeftX(), Controle.DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftY(), Controle.DEADBAND), () -> driver.getRawAxis(2));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    Trigger rightTrigger = driver.button(3);

    driver.button(2).onTrue(Commands.runOnce(swerve::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setHeadingCorrection() {
    swerve.setHeadingCorrection(true);
  }

  public void setMotorBrake() {
    swerve.setMotorBrake();
  }

}
