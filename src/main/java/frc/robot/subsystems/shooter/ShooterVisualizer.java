package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.shooter.FlyWheel.FlyWheelIO.FlyWheelIOInputs;
import frc.robot.subsystems.shooter.Pivot.PivotIO.PivotIOInputs;
import frc.robot.subsystems.shooter.Turret.TurretIO.TurretIOInputs;
import frc.robot.util.FieldConstants;

public class ShooterVisualizer {

    private boolean CALIBRATION_MODE = false;

    private final Mechanism2d mechanism;
    private final MechanismRoot2d root;
    private final MechanismLigament2d turretLigament;
    private final MechanismLigament2d pivotLigament;
    private final MechanismLigament2d flywheelLigament;

    private final Transform3d robotToTurret = new Transform3d(
        new Translation3d(0.0, 0.0, 0.2), new Rotation3d(0, 0, 0));
    private final Transform3d turretToPivot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.3), new Rotation3d(0, 0, 0));
    private final Transform3d robotToIntake = new Transform3d(
        new Translation3d(0.35, 0.0, 0.1), new Rotation3d(0, 0, 0));

    private final PivotIOInputs pivotInputs;
    private final FlyWheelIOInputs flywheelInputs;
    private final TurretIOInputs turretInputs;

    private static final int MAX_BALLS = 3;
    private static final double SHOOT_INTERVAL = 1.0 / 3.0;
    private static final double BALL_FLIGHT_TIME = 1.0;
    private static final double HUB_HEIGHT_METERS = 2.4;
    private static final double PARABOLA_ARC_HEIGHT = 1.5;
    private static final int PARABOLA_POINTS = 50;

    private final Translation3d[] ballLaunchPos = new Translation3d[MAX_BALLS];
    private final Translation2d[] ballTargetHub = new Translation2d[MAX_BALLS];
    private final double[] ballProgress = new double[MAX_BALLS];
    private final boolean[] ballInAir = new boolean[MAX_BALLS];
    private double shootCooldown = 0.0;

    public ShooterVisualizer(PivotIOInputs pivotInputs, FlyWheelIOInputs flywheelInputs, TurretIOInputs turretInputs) {
        this.pivotInputs = pivotInputs;
        this.flywheelInputs = flywheelInputs;
        this.turretInputs = turretInputs;

        this.mechanism = new Mechanism2d(3.0, 3.0);
        this.root = mechanism.getRoot("ShooterBase", 1.5, 0.5);
        this.turretLigament = root.append(new MechanismLigament2d("Turret", 0.5, 90, 6, new Color8Bit(Color.kBlue)));
        this.pivotLigament = turretLigament.append(new MechanismLigament2d("Pivot", 1.0, 45, 4, new Color8Bit(Color.kYellow)));
        this.flywheelLigament = pivotLigament.append(new MechanismLigament2d("Flywheel", 0.2, 0, 10, new Color8Bit(Color.kRed)));

        for (int i = 0; i < MAX_BALLS; i++) {
            ballLaunchPos[i] = new Translation3d();
            ballTargetHub[i] = new Translation2d();
            ballProgress[i] = 0.0;
            ballInAir[i] = false;
        }
    }

    public void update(Pose2d robotPose, boolean isShooting, double intakeAngleRads) {

        turretLigament.setAngle(Units.radiansToDegrees(turretInputs.positionRads) + 90);
        pivotLigament.setAngle(Units.radiansToDegrees(pivotInputs.positionRads));

        double speedRads = Math.abs(flywheelInputs.velocityRadsPerSec);
        if (speedRads > 50.0) flywheelLigament.setColor(new Color8Bit(Color.kLimeGreen));
        else if (speedRads > 10.0) flywheelLigament.setColor(new Color8Bit(Color.kOrange));
        else flywheelLigament.setColor(new Color8Bit(Color.kRed));
        SmartDashboard.putData("Shooter/Mechanism2d", mechanism);

        if (CALIBRATION_MODE) {
            Pose3d zero = new Pose3d();
            Logger.recordOutput("Sim/Robot3D", new Pose3d[]{ zero, zero, zero });
            return;
        }

        Pose3d robotPose3d = new Pose3d(robotPose);
        Pose3d origin = new Pose3d();

        Pose3d turretPose = origin
            .transformBy(robotToTurret)
            .transformBy(new Transform3d(new Translation3d(0.2, 0.15, 0.15),
                new Rotation3d(Math.toRadians(90), 0, Math.toRadians(180) - turretInputs.positionRads)));
        Pose3d pivotPose = turretPose
            .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretInputs.positionRads / 4)))
            .transformBy(turretToPivot)
            .transformBy(new Transform3d(new Translation3d(0.09, 0.04, -0.27),
                new Rotation3d(Math.toRadians(180), Math.toRadians(180), Math.toRadians(180))));
        Pose3d intakePose = origin
            .transformBy(robotToIntake)
            .transformBy(new Transform3d(new Translation3d(-0.65, 0.30, 0.13),
                new Rotation3d(0, -intakeAngleRads, 0)));

        Logger.recordOutput("Sim/RobotTurretPose", turretPose);
        Logger.recordOutput("Sim/RobotPivotPose", pivotPose);
        Logger.recordOutput("Sim/RobotIntakePose", intakePose);
        Logger.recordOutput("Sim/RobotPose", robotPose3d);

        Pose3d turretWorldPose = robotPose3d
            .transformBy(robotToTurret)
            .transformBy(new Transform3d(new Translation3d(0.2, 0.15, 0.15),
                new Rotation3d(Math.toRadians(90), 0, Math.toRadians(180) - turretInputs.positionRads)));

        // Select target based on alliance and field position
        boolean isRed = isRedAlliance();
        boolean inSpitZone = isRed ? (robotPose.getX() < 11.272) : (robotPose.getX() > 4.750);

        Translation2d target;
        if (inSpitZone) {
            target = (robotPose.getY() > 4.1)
                ? FieldConstants.FieldPoses.kSpitPointLeft.getTranslation()
                : FieldConstants.FieldPoses.kSpitPointRight.getTranslation();
            if (isRed) target = new Translation2d(16.541 - target.getX(), target.getY());
        } else {
            target = isRed
                ? FieldConstants.FieldPoses.kHubRed
                : FieldConstants.FieldPoses.kHubBlue;
        }

        // Ball cadence
        double dt = 0.02;
        shootCooldown = Math.max(0.0, shootCooldown - dt);
        if (isShooting && shootCooldown <= 0.0) {
            for (int i = 0; i < MAX_BALLS; i++) {
                if (!ballInAir[i]) {
                    ballInAir[i] = true;
                    ballLaunchPos[i] = turretWorldPose.getTranslation();
                    ballTargetHub[i] = target;
                    ballProgress[i] = 0.0;
                    shootCooldown = SHOOT_INTERVAL;
                    break;
                }
            }
        }
        if (!isShooting) shootCooldown = 0.0;

        // Advance each ball along the arc
        Pose3d[] ballPoses = new Pose3d[MAX_BALLS];
        for (int i = 0; i < MAX_BALLS; i++) {
            if (ballInAir[i]) {
                ballProgress[i] += dt / BALL_FLIGHT_TIME;
                if (ballProgress[i] >= 1.0) {
                    ballInAir[i] = false;
                    ballProgress[i] = 1.0;
                }
                ballPoses[i] = interpolateArc(ballLaunchPos[i], ballTargetHub[i], ballProgress[i]);
            } else {
                ballPoses[i] = turretWorldPose;
            }
        }
        Logger.recordOutput("Sim/GamePieces/Fuel", ballPoses);

        Logger.recordOutput("Sim/Parabola", calculateParabola(turretWorldPose.getTranslation(), target));
    }

    private Pose3d interpolateArc(Translation3d launch, Translation2d target, double t) {
        double x = launch.getX() + t * (target.getX() - launch.getX());
        double y = launch.getY() + t * (target.getY() - launch.getY());
        double zLinear = launch.getZ() + t * (HUB_HEIGHT_METERS - launch.getZ());
        double zArc = PARABOLA_ARC_HEIGHT * Math.sin(Math.PI * t);
        return new Pose3d(new Translation3d(x, y, zLinear + zArc), new Rotation3d());
    }

    private Pose3d[] calculateParabola(Translation3d launch, Translation2d target) {
        Pose3d[] points = new Pose3d[PARABOLA_POINTS];
        for (int i = 0; i < PARABOLA_POINTS; i++) {
            double t = (double) i / (PARABOLA_POINTS - 1);
            points[i] = interpolateArc(launch, target, t);
        }
        return points;
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}