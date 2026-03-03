package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.intakeConstants;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIOInputsAutoLogged;
import frc.robot.subsystems.shooter.Pivot.PivotIO;
import frc.robot.subsystems.shooter.Pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.shooter.Turret.TurretIO;
import frc.robot.subsystems.shooter.Turret.TurretIOInputsAutoLogged;

public class Shooter extends SubsystemBase {

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final FlyWheelIO flywheelIO;
    private final FlyWheelIOInputsAutoLogged flywheelInputs = new FlyWheelIOInputsAutoLogged();

    // Adicionei aqui o visualizador do shooter
    private final ShooterVisualizer visualizer;
    
    // Adicionei as variáveis necessárias para receber o ângulo do Intake
    private DoubleSupplier intakeAngleSupplier = () -> 0.0;
    
    // CORREÇÃO: Faltava fechar a chave } deste método!
    public void setIntakeAngleSupplier(DoubleSupplier supplier) {
        this.intakeAngleSupplier = supplier;
    }
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Translation2d> targetSupplier;
    private Supplier<ChassisSpeeds> robotVelocitySupplier;

    private double currentTurretTarget = 0.0;
    private double currentPivotTarget = 68.89;
    private double kPivotOffset = 27.0;

    private double currentFlywheelTargetRpm = 0.0;
    private double currentFeederTargetRpm = 0.0;
    private double currentCentrifugeTargetRpm = 0.0;
    private Translation2d lastTargetLocation = null;

    public static double kMinPivotAngle = 55.0;
    private double kMaxPivotAngle = 68.89;

    public double kFeederShootRpm = 5500.0;
    public double kCentrifugeShootRpm = 6000.0;

    private double kTargetHeightRelative = 2.1;
    private final double kSpitHeight = 0.3;
    private double currentTargetHeight = kTargetHeightRelative;

    private double kTurretToleranceDeg = 1.0;
    private double kPivotToleranceDeg = 1.0;

    private double kFlywheelToleranceRpm = 20.0;

    private double kMinTurretAngle = -110.0;
    private double kMaxTurretAngle = 17.0;

    private double kRpmSlope = 225.0;
    private double kRpmIntercept = 2000.00;
    private double kMaxSafeRpm = 6000.0;

    private final double kGravity = 9.81;
    private final double kWheelRadiusMeters = Units.inchesToMeters(2.0);

    public double calculatedAutoAimRpm = 0.0;

    public static boolean hasReachedSpeed = false;
    private boolean autoAimEnabled = false;
    private boolean flywheelWithinTolerance = false;
    private boolean isFirstTimeRunning = true;

    private boolean testModeEnabled = false;
    private double testManualRpm = 4000.0;

    private double kTurretManualSpeed = 1.0;
    private double kPivotManualSpeed = 0.5;

    private double kYoffset = 0.235;
    private double kXoffset = 0.115;

    private final Timer intakeAgitatorTimer = new Timer();
    private boolean isIntakeAgitatorDown = false;

    private final Timer timer = new Timer();
    public static boolean isFirstPushingTrigger = true;
    public static boolean isShooting = false;

    // Variáveis de Tuning do Shoot-on-the-fly
    private double kTimeOfFlightMultiplier = 1.9; // Compensa a resistência do ar
    private double kSystemLatencySeconds = 0.9; // Compensa atrasos mecânicos

    public Shooter(TurretIO turretIO, PivotIO pivotIO, FlyWheelIO flywheelIO) {
        this.turretIO = turretIO;
        this.pivotIO = pivotIO;
        this.flywheelIO = flywheelIO;
        this.visualizer = new ShooterVisualizer(pivotInputs, flywheelInputs, turretInputs);


        SmartDashboard.putNumber("Tuning/Shooter/MinPivotAngle", kMinPivotAngle);

        // Inicializa os valores no SmartDashboard para calibração
        SmartDashboard.putNumber("Tuning/Shooter/ToF_Multiplier", kTimeOfFlightMultiplier);
        SmartDashboard.putNumber("Tuning/Shooter/System_Latency", kSystemLatencySeconds);
    }

    public void setupAutoAimReferences(Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.targetSupplier = targetSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
    }

    private double convertRpmToMps(double rpm) {
        double angularVelocityRadPerSec = rpm * (2.0 * Math.PI / 60.0);
        double wheelSurfaceSpeedMps = angularVelocityRadPerSec * kWheelRadiusMeters;
        double noteExitSpeedMps = wheelSurfaceSpeedMps;
        return noteExitSpeedMps;
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);
        pivotIO.updateInputs(pivotInputs);
        flywheelIO.updateInputs(flywheelInputs);

        Logger.processInputs("Shooter/Turret", turretInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheel", flywheelInputs);

        SmartDashboard.putNumber("Turret/CurrentAngle", getTurretPosition());
        SmartDashboard.putNumber("Pivot/CurrentAngle", getPivotPosition());

        kTimeOfFlightMultiplier = SmartDashboard.getNumber("Tuning/Shooter/ToF_Multiplier", kTimeOfFlightMultiplier);
        kSystemLatencySeconds = SmartDashboard.getNumber("Tuning/Shooter/System_Latency", kSystemLatencySeconds);

        if (autoAimEnabled && robotPoseSupplier != null && targetSupplier != null && robotVelocitySupplier != null) {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d realTargetLocation = targetSupplier.get();
            ChassisSpeeds robotSpeeds = robotVelocitySupplier.get();

            boolean inHomeZone = isInHomeZone(robotPose);
            currentTargetHeight = inHomeZone ? kTargetHeightRelative : kSpitHeight;
            SmartDashboard.putNumber("Shooter/CurrentTargetHeight", currentTargetHeight);

            Translation2d fieldVelocity = new Translation2d(
                    robotSpeeds.vxMetersPerSecond,
                    robotSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());

            double staticDist = getDistanceToTarget(robotPose, realTargetLocation);
            double staticTargetRpm = testModeEnabled ? testManualRpm : (kRpmSlope * staticDist) + kRpmIntercept;
            staticTargetRpm = MathUtil.clamp(staticTargetRpm, 0, kMaxSafeRpm);

            double staticMps = convertRpmToMps(staticTargetRpm);
            double staticPivot = calculatePivotAngleNumeric(staticDist, staticMps);

            double horizontalMps = staticMps * Math.cos(Math.toRadians(staticPivot));
            double baseTimeOfFlight = staticDist / Math.max(horizontalMps, 0.1);

            double timeOfFlight = (baseTimeOfFlight * kTimeOfFlightMultiplier) + kSystemLatencySeconds;

            Translation2d virtualTargetLocation = new Translation2d(
                    realTargetLocation.getX() - (fieldVelocity.getX() * timeOfFlight),
                    realTargetLocation.getY() - (fieldVelocity.getY() * timeOfFlight));

            Logger.recordOutput("Shooter/RealTargetPose", new Pose2d(realTargetLocation, new Rotation2d()));
            Logger.recordOutput("Shooter/VirtualTargetPose", new Pose2d(virtualTargetLocation, new Rotation2d()));

            double dynamicDist = getDistanceToTarget(robotPose, virtualTargetLocation);
            SmartDashboard.putNumber("Calibration/TEST_DistanceToTarget", dynamicDist);

            double rawTargetRpm = testModeEnabled ? testManualRpm : (kRpmSlope * dynamicDist) + kRpmIntercept;
            rawTargetRpm = MathUtil.clamp(rawTargetRpm, 0, kMaxSafeRpm);
            double targetRpm = rawTargetRpm;

            double dynamicMps = convertRpmToMps(targetRpm);
            double targetPivot = calculatePivotAngleNumeric(dynamicDist, dynamicMps);
            double targetTurret = calculateTurretAngle(robotPose, virtualTargetLocation);

            setTurretSetpoint(targetTurret);
            setPivotPosition(targetPivot);
            setFlywheelVelocity(targetRpm);

            calculatedAutoAimRpm = targetRpm;

            if (inHomeZone) {
                setFlywheelVelocity(calculatedAutoAimRpm);
            } else {
                if (!isShooting) {
                    setFlywheelVelocity(0);
                } else {
                    setFlywheelVelocity(calculatedAutoAimRpm);
                }
            }
        }

        currentTurretTarget = MathUtil.clamp(currentTurretTarget, kMinTurretAngle, kMaxTurretAngle);
        turretIO.runSetpoint(Degrees.of(currentTurretTarget));

        currentPivotTarget = MathUtil.clamp(currentPivotTarget, kMinPivotAngle, kMaxPivotAngle);
        pivotIO.runSetpoint(Degrees.of(currentPivotTarget));

        if (currentFlywheelTargetRpm < 10 && currentFeederTargetRpm < 10 && currentCentrifugeTargetRpm < 10) {
            flywheelIO.stop();
        } else {
            if (currentFlywheelTargetRpm >= 10)
                flywheelIO.runVelocity(RPM.of(currentFlywheelTargetRpm));

            if (currentFeederTargetRpm < 10)
                flywheelIO.runFeederVolts(0.0);
            else
                flywheelIO.runFeederVelocity(RPM.of(currentFeederTargetRpm));

            if (currentCentrifugeTargetRpm < 10)
                flywheelIO.runCentrifugeVolts(0.0);
            else
                flywheelIO.runCentrifugeVelocity(RPM.of(currentCentrifugeTargetRpm));
             
        }
        Pose2d currentPose = (robotPoseSupplier != null) ? robotPoseSupplier.get() : new Pose2d();
        
        // Pega o ângulo atual através da função que vamos fornecer no RobotContainer
        double currentIntakeAngleRads = intakeAngleSupplier.getAsDouble(); 

        // CORREÇÃO: Usando a variável currentIntakeAngleRads
        visualizer.update(currentPose, isShooting, currentIntakeAngleRads);
        //System.out.println(currentIntakeAngleRads);
    }

    public void setFlywheelVelocity(double rpm) {
        this.currentFlywheelTargetRpm = rpm;
    }

    public void runFeeder(double rpm) {
        this.currentFeederTargetRpm = rpm;
    }

    public void runCentrifuge(double rpm) {
        this.currentCentrifugeTargetRpm = rpm;
    }

    public double getFlywheelRpm() {
        return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadsPerSec);
    }

    public boolean isFlywheelAtSpeed() {
        if (currentFlywheelTargetRpm < 500) {
            flywheelWithinTolerance = false;
            return false;
        }
        double error = Math.abs(getFlywheelRpm() - currentFlywheelTargetRpm);
        if (!flywheelWithinTolerance) {
            if (error <= kFlywheelToleranceRpm)
                flywheelWithinTolerance = true;
        } else {
            if (error > kFlywheelToleranceRpm * 4.0)
                flywheelWithinTolerance = false;
        }
        return flywheelWithinTolerance;
    }

    public Command runShootSequence(double manualFlywheelTargetRpm, double feederTargetRpm,
            double centrifugeTargetRpm) {
        return this.run(() -> {

            isShooting = true;

            if (isFirstPushingTrigger) {
                isFirstPushingTrigger = false;
                intakeConstants.kIntakePushing = false;
                timer.restart();
            }

            if (!hasReachedSpeed && isFlywheelAtSpeed()) {
                hasReachedSpeed = true;
            }

            if (hasReachedSpeed) {
                runFeeder(feederTargetRpm);
                runCentrifuge(centrifugeTargetRpm);
            }

            else {
                runFeeder(0.0);
                runCentrifuge(0.0);
            }

            if (timer.hasElapsed(intakeConstants.KTimerResetSeconds)) {
                isFirstPushingTrigger = true;
                intakeConstants.kIntakePushing = true;
            }

        }).beforeStarting(() -> {
            hasReachedSpeed = false;
            flywheelWithinTolerance = false;
            isFirstPushingTrigger = true;

            timer.reset();

        }).finallyDo(() -> {
            hasReachedSpeed = false;
            flywheelWithinTolerance = false;
            isFirstPushingTrigger = true;
            isShooting = false;
            stop();
        });
    }

    public void reverseSystem() {
        flywheelIO.runFeederVelocity(RPM.of(-3000));
        flywheelIO.runCentrifugeVelocity(RPM.of(-3000));
    }

    public void reverseOffSystem() {
        flywheelIO.runFeederVelocity(RPM.of(0));
        flywheelIO.runCentrifugeVelocity(RPM.of(0));
    }

    public Command shootCommand(Intake intake) {
        return runShootSequence(calculatedAutoAimRpm, kFeederShootRpm, kCentrifugeShootRpm);
    }

    public void shootCommandAuto(Intake intake, double calculatedAutoAimRpm, double feederTargetRpm,
            double centrifugeTargetRpm) {

        if (isFirstTimeRunning) {
            hasReachedSpeed = false;
            flywheelWithinTolerance = false;
            isFirstPushingTrigger = true;

            timer.reset();
            isFirstTimeRunning = false;
        }

        double activeRpm = calculatedAutoAimRpm;
        setFlywheelVelocity(activeRpm);
        System.out.println("Velocidade: " + activeRpm);
        isShooting = true;

        if (isFirstPushingTrigger) {
            isFirstPushingTrigger = false;
            intakeConstants.kIntakePushing = false;
            timer.restart();
        }

        if (!hasReachedSpeed && isFlywheelAtSpeed())
            hasReachedSpeed = true;

        if (hasReachedSpeed) {
            runFeeder(feederTargetRpm);
            runCentrifuge(centrifugeTargetRpm);
        } else {
            runFeeder(0.0);
            runCentrifuge(0.0);
        }

        if (timer.hasElapsed(intakeConstants.KTimerResetSeconds)) {
            isFirstPushingTrigger = true;
            intakeConstants.kIntakePushing = true;
        }
    }

    public double getDistanceToTarget(Pose2d robotPose, Translation2d targetLocation) {
        return robotPose.getTranslation().getDistance(targetLocation);
    }

    public double calculateTurretAngle(Pose2d robotPose, Translation2d targetLocation) {
        double angleRad = Math.atan2(targetLocation.getY() - (robotPose.getY() + kYoffset),
                targetLocation.getX() - (robotPose.getX()) + kXoffset);
        return MathUtil.inputModulus(robotPose.getRotation().getDegrees() - Math.toDegrees(angleRad), -180, 180);
    }

    public double calculatePivotAngleNumeric(double distanceMeters, double velocityMPS) {
        double v2 = Math.pow(velocityMPS, 2);

        double discriminant = Math.pow(v2, 2)
                - kGravity * (kGravity * Math.pow(distanceMeters, 2) + 2 * currentTargetHeight * v2);

        if (discriminant < 0)
            return kMaxPivotAngle;

        double safeDistanceForMath = Math.max(distanceMeters, 0.1);
        double idealAngle = Math
                .toDegrees(Math.atan((v2 - Math.sqrt(discriminant)) / (kGravity * safeDistanceForMath)));

        return idealAngle;
    }

    public void setTurretSetpoint(double degrees) {
        this.currentTurretTarget = degrees;
    }

    public double getTurretPosition() {
        return Units.radiansToDegrees(turretInputs.positionRads);
    }

    public void setPivotPosition(double degreesReal) {
        this.currentPivotTarget = degreesReal + kPivotOffset;
    }

    public double getPivotPosition() {
        return Units.radiansToDegrees(pivotInputs.positionRads);
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }

    public void stop() {
        currentFlywheelTargetRpm = 0.0;
        currentFeederTargetRpm = 0.0;
        currentCentrifugeTargetRpm = 0.0;
        flywheelIO.stop();
    }

    public void resetTurretEncoder() {
        turretIO.resetEncoder();
    }

    public void resetPivotEncoder() {
        pivotIO.resetEncoder();
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public Command manualTurretCommand(boolean isRight) {
        return this.run(() -> {
            if (!autoAimEnabled) {
                currentTurretTarget += isRight ? kTurretManualSpeed : -kTurretManualSpeed;
                System.out.printf("TURRET DEBUG >> Alvo: %.2f | Real: %.2f%n",
                        currentTurretTarget, getTurretPosition());
            }
        }).withName("ManualTurret");
    }

    public Command manualPivotCommand(boolean isUp) {
        return this.run(() -> {
            if (!autoAimEnabled) {
                currentPivotTarget += isUp ? kPivotManualSpeed : -kPivotManualSpeed;
                System.out.printf("PIVOT DEBUG >> Alvo: %.2f | Real: %.2f%n",
                        currentPivotTarget, getPivotPosition());
            }
        }).withName("ManualPivot");
    }

    public boolean isTurretAtTarget() {
        return Math.abs(getTurretPosition() - currentTurretTarget) <= kTurretToleranceDeg;
    }

    public boolean isPivotAtTarget() {
        return Math.abs(getPivotPosition() - currentPivotTarget) <= kPivotToleranceDeg;
    }

    private boolean isInHomeZone(Pose2d robotPose) {
        var allianceOpt = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (allianceOpt.isEmpty())
            return false;

        boolean isRed = allianceOpt.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        double x = robotPose.getX();

        if (isRed) {
            return x > 11.2;
        } else {
            return x < 5.2;
        }
    }

    public boolean isReadyToShoot() {
        return autoAimEnabled &&
                isTurretAtTarget() &&
                isPivotAtTarget();
    }
}