/*
 * Copyright (c) 2024 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125;

import static io.github.tigerbotics7125.Constants.Arm.ArmState.*;
import static io.github.tigerbotics7125.Constants.DriveTrain.ControlType.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.tigerbotics7125.Constants.DriveTrain.ControlType;
import io.github.tigerbotics7125.subsystems.Arm;
import io.github.tigerbotics7125.subsystems.Drivetrain;
import io.github.tigerbotics7125.subsystems.Intake;
import io.github.tigerbotics7125.subsystems.Shooter;
import java.util.Map;

public class Robot extends TimedRobot {

  // Operator Interface
  private CommandXboxController m_driver, m_operator;

  // Subsystems
  private Drivetrain m_drivetrain;
  private Intake m_intake;
  private Shooter m_shooter;
  private Arm m_arm;

  // Choosers
  private SendableChooser<Command> m_autoChooser;
  private SendableChooser<ControlType> m_driveChooser;

  @Override
  public void robotInit() {
    // Operator Interfaces
    m_driver = new CommandXboxController(Constants.HID.kDriverControllerPort);
    m_operator = new CommandXboxController(Constants.HID.kOperatorControllerPort);

    // Subystems
    m_drivetrain = new Drivetrain();
    m_intake = new Intake();
    m_shooter = new Shooter();
    m_arm = new Arm();

    // Choosers
    m_autoChooser = AutoBuilder.buildAutoChooser("Test");
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    m_driveChooser = new SendableChooser<>();
    m_driveChooser.setDefaultOption(CURVE_ROCKETLEAGUE.name(), CURVE_ROCKETLEAGUE);
    m_driveChooser.addOption(ARCADE_ROCKETLEAGUE.name(), ARCADE_ROCKETLEAGUE);
    m_driveChooser.addOption(CURVE.name(), CURVE);
    m_driveChooser.addOption(ARCADE.name(), ARCADE);
    SmartDashboard.putData("Drivetrain Control Type Chooser", m_driveChooser);

    // Driver Camera
    // CameraServer.startAutomaticCapture();

    configureDefaultCommands();
    configureTriggers();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    Command autoCommand = m_autoChooser.getSelected();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Make sure autonomous commands are canceled for teleop
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  /** Configure triggers and button bindings. */
  public void configureTriggers() {
    // Set arm and drivetrain motors to break mode when you press the USER button on the rio.
    new Trigger(RobotController::getUserButton)
        .onTrue(
            m_drivetrain.setIdleMode(IdleMode.kBrake).andThen(m_arm.setIdleMode(IdleMode.kBrake)));

    m_operator.rightBumper().onTrue(m_intake.intake());
    m_operator.rightBumper().onFalse(m_intake.outtake(m_operator::getRightTriggerAxis));
    m_operator.leftBumper().onTrue(m_shooter.shootNote(m_intake));
    // TODO: consider having the shooter idle at a lower speed as opposed to disabling
    m_operator.leftBumper().onFalse(m_shooter.disable());
    m_operator.a().onTrue(m_arm.disable());
    m_operator.b().onTrue(m_arm.pidControl(INTAKE));
    m_operator.x().onTrue(m_arm.pidControl(AMP));
    m_operator.y().onTrue(m_arm.pidControl(SPEAKER));
    // m_operator.leftStick().onTrue(m_arm.resetEncoder());
  }

  /**
   * Configure all subsystems default commands, which are commands that run when nothing else is
   * scheduled for the subsystem.
   */
  public void configureDefaultCommands() {
    // Default drive command which selects from the default of the drivetrain control type chooser.
    m_drivetrain.setDefaultCommand(
        Commands.select(
            Map.of(
                ControlType.ARCADE,
                m_drivetrain.arcadeDrive(m_driver::getLeftY, m_driver::getRightX, () -> true),
                ControlType.ARCADE_ROCKETLEAGUE,
                m_drivetrain.arcadeDrive(
                    () -> m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis(),
                    m_driver::getLeftX,
                    () -> true),
                ControlType.CURVE,
                m_drivetrain.curvatureDrive(m_driver::getLeftY, m_driver::getRightX, () -> true),
                ControlType.CURVE_ROCKETLEAGUE,
                m_drivetrain.curvatureDrive(
                    () -> m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis(),
                    m_driver::getLeftX,
                    () -> true)),
            m_driveChooser::getSelected));
    // Updates the drivetrain's default command if the chosen control type changes.
    m_driveChooser.onChange(
        controlType ->
            m_drivetrain.setDefaultCommand(
                switch (controlType) {
                  case ARCADE -> m_drivetrain.arcadeDrive(
                      m_driver::getLeftY, m_driver::getRightX, () -> true);
                  case ARCADE_ROCKETLEAGUE -> m_drivetrain.arcadeDrive(
                      () -> m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis(),
                      m_driver::getLeftX,
                      () -> true);
                  case CURVE -> m_drivetrain.curvatureDrive(
                      m_driver::getLeftY, m_driver::getRightX, () -> true);
                  case CURVE_ROCKETLEAGUE -> m_drivetrain.curvatureDrive(
                      () -> m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis(),
                      m_driver::getLeftX,
                      () -> true);
                }));
    // Stop motors when we aren't scheduling any commands.
    m_intake.setDefaultCommand(m_intake.disable());
    m_shooter.setDefaultCommand(m_shooter.disable());
    m_arm.setDefaultCommand(m_arm.disable());
  }
}
