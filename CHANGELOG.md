# CommandBased Changelog

1. [HID Updates](#hid-updates)
2. [Setup CommandScheduler](#setup-commandscheduler)
3. [Drivetrain Subsystem](#basic-drivetrain-subsystem)
    - [Drivetrain Commands](#drivetrain-commands)
4. [Intake Subsystem](#intake-subsystem)
5. [Shooter Subsystem](#shooter-subsystem)
6. [Arm Subsystem](#arm-subsystem)

## HID updates
Add HID constants to `Constants.java`.
```java
public final class Constants {
    public final static class HID {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }
}
```
Use HID constants in `Robot.java`.
```java
public class Robot extends TimedRobot {
    private XboxController mXboxDrive = new XboxController(Constants.HID.kDriverControllerPort);
    private XboxController mXboxOperator = new XboxController(Constants.HID.kOperatorControllerPort);
}
```
## Setup CommandScheduler
Add CommandScheduler to `Robot.java`.
```java
public class Robot extends TimedRobot {
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // Make sure autonomous commands are canceled for teleop
        CommandScheduler.getInstance().cancelAll();
    }
}
```
## Drivetrain Subsystem
Convert basic drivetrain code to a subsystem in `Drivetrain.java`.
```java
public class Drivetrain extends SubsystemBase {
    private CANSparkMax frontLeft =
            new CANSparkMax(Constants.DriveTrain.kFrontLeftID, Constants.DriveTrain.kMotorType);
    private CANSparkMax frontRight =
            new CANSparkMax(Constants.DriveTrain.kFrontRightID, Constants.DriveTrain.kMotorType);
    private CANSparkMax backLeft =
            new CANSparkMax(Constants.DriveTrain.kBackLeftID, Constants.DriveTrain.kMotorType);
    private CANSparkMax backRight =
            new CANSparkMax(Constants.DriveTrain.kBackRightID, Constants.DriveTrain.kMotorType);

    public Drivetrain() {
        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        frontRight.setInverted(true);
        // No need to tell backRight to invert, it's a follower.
    }

    private void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        Timer.delay(.02);

        // TODO motor configs, we need to do this but we can later.

        motor.burnFlash();
        Timer.delay(.02);
    }

    public Command arcadeDrive(
            DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier squareInputs) {
        return run(
                () -> {
                    WheelSpeeds ws =
                            DifferentialDrive.arcadeDriveIK(
                                    xSpeed.getAsDouble(),
                                    zRotation.getAsDouble(),
                                    squareInputs.getAsBoolean());
                    frontLeft.set(ws.left);
                    frontRight.set(ws.right);
                });
    }

    public Command curvatureDrive(
            DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier allowTurnInPlace) {
        return run(
                () -> {
                    WheelSpeeds ws =
                            DifferentialDrive.curvatureDriveIK(
                                    xSpeed.getAsDouble(),
                                    zRotation.getAsDouble(),
                                    allowTurnInPlace.getAsBoolean());
                    frontLeft.set(ws.left);
                    frontRight.set(ws.right);
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/DT/Left", frontLeft.get());
        SmartDashboard.putNumber("/DT/Right", frontRight.get());
    }
}
```
Add drivetrain constants to `Constants.java`.
```java
public final Constants {
    public static final class DriveTrain {
        public static final MotorType kMotorType = MotorType.kBrushed;
        public static final int kFrontLeftID = 1;
        public static final int kFrontRightID = 2;
        public static final int kBackLeftID = 3;
        public static final int kBackRightID = 4;
    }
}
```
Previous drivetrain code was removed from `Robot.java` and the subsystem object was added.

Note that at this point, the drivetrain has no Command controlling it.
```java
public class Robot extends TimedRobot {
    private Drivetrain m_drivetrain = new Drivetrain();
}
```
### Drivetrain Commands

This is relatively complex, I can explain further if necessary.

Add `ControlType` enum to drivetrain constants in `Constants.java`.
```java
public final class Constants {
    public static final class DriveTrain {
        public enum ControlType {
            ARCADE,
            ARCADE_ROCKETLEAGUE,
            CURVE,
            CURVE_ROCKETLEAGUE;
        }
    }
}
```
Add a `ControlType` chooser to `Robot.java`.

Set default command of drivetrain to the default of the chooser automatically.

Set an onChange listener to the chooser to change the default command when a new `ControlType` is selected.
```java
public class Robot extends TimedRobot {
    SendableChooser<Constants.DriveTrain.ControlType> m_driveControlChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        m_driveControlChooser.setDefaultOption(ControlType.CURVE_ROCKETLEAGUE.name(), ControlType.CURVE_ROCKETLEAGUE);
        for (ControlType controlType : ControlType.values()) {
            if (controlType.equals(ControlType.CURVE_ROCKETLEAGUE)) continue;
            m_driveControlChooser.addOption(controlType.name(), controlType);
        }

        m_drivetrain.setDefaultCommand(
                Commands.select(
                        Map.of(
                                ControlType.ARCADE,
                                m_drivetrain.arcadeDrive(
                                        m_driver::getLeftY, m_driver::getRightX, () -> true),
                                ControlType.ARCADE_ROCKETLEAGUE,
                                m_drivetrain.arcadeDrive(
                                        () ->
                                                m_driver.getRightTriggerAxis()
                                                        - m_driver.getLeftTriggerAxis(),
                                        m_driver::getLeftX,
                                        () -> true),
                                ControlType.CURVE,
                                m_drivetrain.curvatureDrive(
                                        m_driver::getLeftY, m_driver::getRightX, () -> true),
                                ControlType.CURVE_ROCKETLEAGUE,
                                m_drivetrain.curvatureDrive(
                                        () ->
                                                m_driver.getRightTriggerAxis()
                                                        - m_driver.getLeftTriggerAxis(),
                                        m_driver::getLeftX,
                                        () -> true)),
                        m_driveControlChooser::getSelected));
        m_driveControlChooser.onChange(
                controlType ->
                        m_drivetrain.setDefaultCommand(
                                switch (controlType) {
                                    case ARCADE -> m_drivetrain.arcadeDrive(
                                            m_driver::getLeftY, m_driver::getRightX, () -> true);
                                    case ARCADE_ROCKETLEAGUE -> m_drivetrain.arcadeDrive(
                                            () ->
                                                    m_driver.getRightTriggerAxis()
                                                            - m_driver.getLeftTriggerAxis(),
                                            m_driver::getLeftX,
                                            () -> true);
                                    case CURVE -> m_drivetrain.curvatureDrive(
                                            m_driver::getLeftY, m_driver::getRightX, () -> true);
                                    case CURVE_ROCKETLEAGUE -> m_drivetrain.curvatureDrive(
                                            () ->
                                                    m_driver.getRightTriggerAxis()
                                                            - m_driver.getLeftTriggerAxis(),
                                            m_driver::getLeftX,
                                            () -> true)
                                }));
    }
}
```
## Intake Subsystem
Convert intake (minus shooter) to subsystem in `Intake.java`.
```java
public class Intake extends SubsystemBase {

    private CANSparkMax m_intake = new CANSparkMax(Constants.Intake.kIntakeID, Constants.Intake.kMotorType);

    public Intake() {
        configMotor(m_intake);
    }

    private void configMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        Timer.delay(.02);

        // TODO motor configs, we need to do this but we can later.

        m_intake.setInverted(Constants.Intake.kInverted);

        motor.burnFlash();
        Timer.delay(.02);
    }

    public Command disable() {
        return run(m_intake::disable);
    }

    public Command intake() {
        return run(() -> m_intake.set(Constants.Intake.kIntakeSpeed));
    }

    public Command outtake(DoubleSupplier axis) {
        return run(() -> m_intake.set(MathUtil.interpolate(0, Constants.Intake.kMaxOutakeSpeed, axis.getAsDouble())));
    }

    public Command feedShooter() {
        return run(() -> m_intake.set(Constants.Intake.kFeedSpeed));
    }
}
```
Add intake constants to `Constants.java`.
```java
public final class Constants{
    public static final class Intake {
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final int kIntakeID = 5;

        public static final boolean kInverted = false;

        public static final double kIntakeSpeed = 0.5;
        public static final double kFeedSpeed = 1D;
        public static final double kMaxOutakeSpeed = -0.25;
    }
}
```
Add intake subsystem & assign its commands in `Robot.java`.

```java
public class Robot extends TimedRobot {
    private Intake m_intake = new Intake();

    @Override
    public void robotInit() {
        m_intake.setDefaultCommand(m_intake.disable());

        m_operator.rightBumper().onTrue(m_intake.intake());
        m_operator.rightBumper().onFalse(m_intake.outtake(m_operator::getRightTriggerAxis));
    }
}
```
## Shooter Subsystem

Convert shooter to subsystem in `Shooter.java`.

```java
public class Shooter extends SubsystemBase {

    private CANSparkMax m_left =
            new CANSparkMax(Constants.Shooter.kLeftID, Constants.Shooter.kMotorType);
    private CANSparkMax m_right =
            new CANSparkMax(Constants.Shooter.kRightID, Constants.Shooter.kMotorType);

    private PIDController m_PID =
            new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);

    private RelativeEncoder m_encoder = m_left.getEncoder();

    public Shooter() {
        configureMotor(m_left);
        configureMotor(m_right);

        m_left.setInverted(Constants.Shooter.kInverted);
        m_right.follow(m_left);

        m_PID.setTolerance(0, Constants.Shooter.kPIDTolerance);
    }

    private void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        Timer.delay(.02);

        // TODO motor configs, we need to do this but we can later.

        motor.burnFlash();
        Timer.delay(.02);
    }

    public Trigger isShooterReady() {
        Trigger atSetpoint = new Trigger(m_PID::atSetpoint);
        Trigger setpointValid = new Trigger(() -> m_PID.getSetpoint() != 0);
        return setpointValid.and(atSetpoint);
    }

    public Command disable() {
        return run(m_left::stopMotor);
    }

    public Command prepShooter() {
        return runOnce(() -> m_PID.setSetpoint(Constants.Shooter.kShootRPM));
    }

    public Command shootNote(Intake intake) {
        return prepShooter()
                .andThen(Commands.waitUntil(isShooterReady()))
                .andThen(intake.feedShooter().withTimeout(1));
    }

    public Command pidControl() {
        return run(() -> {
            double pidContribution = 12D * m_PID.calculate(m_encoder.getVelocity());
            m_left.setVoltage(pidContribution + Constants.Shooter.kFF);
        });
    }

    /**
     * TODO seth: create a method to keep the shooter at a constant but slightly
     * less velocity than our shooter velocity. This should be the new default method.
     * Eventually this should consider our distance from the speaker so that it automatically
     * ramps up to shooting speed when we get closer. Not entirely relevant but also should look
     * into using the InterpolatingMap or whatever its called, and some data we can create to find values
     * for shooting from a distance.
     */

    @Override
    public void periodic() {}
}
```
Add shooter constants to `Constants.java`.

```java
public final class Constants {
    public static final class Shooter {
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final int kLeftID = 6;
        public static final int kRightID = 7;

        public static final boolean kInverted = true;

        public static final double kP = 0.0004;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0.00017;
        public static final double kPIDTolerance = 300; // rpm

        public static final double kShootRPM = 5700;
    }
}
```
Add shooter subsystem & assign its commands in `Robot.java`.

```java
public class Robot extends TimedRobot {
    private Shooter m_shooter = new Shooter();

    public void robotInit() {
        m_shooter.setDefaultCommand(m_shooter.disable());
        m_operator.leftBumper().onTrue(m_shooter.shootNote(m_intake));
    }
}
```
## Arm Subsystem

Convert arm to subsystem in `Arm.java`.

```java
public class Arm extends SubsystemBase {

    private CANSparkMax m_left = new CANSparkMax(Constants.Arm.kLeftID, Constants.Arm.kMotorType);
    private CANSparkMax m_right = new CANSparkMax(Constants.Arm.kRightID, Constants.Arm.kMotorType);

    private RelativeEncoder m_encoder = m_left.getEncoder();

    private PIDController m_PID = Constants.Arm.kPID;
    private ArmFeedforward m_feedforward = Constants.Arm.kFF;

    public Arm() {
        configureMotor(m_left);
        configureMotor(m_right);
        m_right.follow(m_left, Constants.Arm.kFollowerInverted);
    }

    private void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        Timer.delay(.02);

        // TODO motor configs, we need to do this but we can later.

        motor.burnFlash();
        Timer.delay(.02);
    }

    public Command disable() {
        return run(m_left::stopMotor);
    }

    public Command setState(ArmState state) {
        return runOnce(() -> m_PID.setSetpoint(state.kPosition));
    }

    public Command pidControl() {
        return run(
                () -> {
                    double pidContribution = 12D * m_PID.calculate(m_encoder.getPosition());
                    double ffContribution = m_feedforward.calculate(m_PID.getSetpoint(), 0);
                    m_left.setVoltage(pidContribution + ffContribution);
                });
    }

    public Command resetEncoder() {
        return runOnce(() -> m_encoder.setPosition(0));
    }

    @Override
    public void periodic() {}
}
```
Add Arm constants to `Constants.java`.
```java
public final class Constants {
    public static final class Arm {
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final int kLeftID = 8;
        public static final int kRightID = 9;

        public static final boolean kFollowerInverted = true;

        public static final double kP = .04;
        public static final double kI = 1e-7; // Integral term should be done with FF instead.
        public static final double kD = .5;
        public static final PIDController kPID = new PIDController(kP, kI, kD);
        // TODO look into sysid to characterize this.
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final ArmFeedforward kFF = new ArmFeedforward(kS, kG, kV);

        public enum ArmState {
            AMP(0D),
            SPEAKER(60D),
            SPEAKERAUTO(63D),
            INTAKE(78D);

            public final double kPosition;

            ArmState(double position) {
                kPosition = position;
            }
        }
    }
}
```
Add arm subsystem & assign its commands in `Robot.java`.
```java
public class Robot extends TimedRobot {
    private Arm m_arm = new Arm();
        public void robotInit() {
            m_arm.setDefaultCommand(m_arm.pidControl());
        m_operator.y().onTrue(m_arm.setState(ArmState.SPEAKER));
        m_operator.x().onTrue(m_arm.setState(ArmState.AMP));
        m_operator.b().onTrue(m_arm.setState(ArmState.INTAKE));
        }
}
```
