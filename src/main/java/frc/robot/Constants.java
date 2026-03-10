
package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

//import java.security.Key;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static boolean ShootAutoEnable = false;

  public static enum Mode {
    REAL, SIM, REPLAY
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMps = 4.0;
    public static final double kMaxAccelMpsSq = 4.0;

    public static final PathConstraints constraints = new PathConstraints(
        kMaxSpeedMps, kMaxAccelMpsSq,
        Units.degreesToRadians(360), Units.degreesToRadians(540));

    public static final PathConstraints constraintsAuto = constraints;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondDriverControllerPort = 1;

    // Driver Controller
    public static final int kResetFrontIdx = 3;
    public static final int kThroughtTrenchIdx = 1;
    public static final int kIntakeFWIdx = 2;
    public static final int kIntakeRollerReverseIdx = 4;

    public static final int kIntakeDriverIdx = 6;
    public static final int KThroughtOtherAllianceTrenchIdx = 7;
    public static final int KLowSpeedIdx = 8;

    // Operator Controller
    public static final int kAutoAimIdx = 5;
    public static final int kStopShootingIdx = 1;
    public static boolean turnShootOffHome = false;
    // public static final int kShootidx = 8;

    public static final int kTurretToLeftPOV = 270;
    public static final int kTurretToRightPOV = 90;
    public static final int kPivotUpPOV = 0;
    public static final int kPivotDownPOV = 180;

    //public static final int kResetTurretEncoderIdx = 4;
    //public static final int kResetPivotIdx = 7;
    public static final int kReverseSystem = 6;
    //public static final int kToggleFlywheel = 1;
    public static final int kIntakeIdxOperador = 2;
    public static final int kIntakeToggleShootingIdx = 3;
    public static final int kIntakeRollerReverseOperadorIdx = 4;
  }

  public static final class LEDConstants {

    public static final int kLEDPort = 9;
    public static final int kTotalLength = 250;
    
    public static final int kBoostStart = 0;
    public static final int kBoostEnd = 15;

    public static final int kShooterStart = kBoostEnd + 1;
    public static final int kShooterEnd = kShooterStart + 24;
    
    public static final int kAllianceStart = 0;
    public static final int kAllianceEnd = kAllianceStart + 138 + 39;

    public static final int kShooterStartSecondary = kAllianceEnd + 1;
    public static final int kShooterEndSecondary = kShooterStartSecondary + 24;
    
    public static final int kBoostStartSecondary = kShooterEndSecondary + 1;
    public static final int kBoostEndSecondary = kBoostStartSecondary + 17;    


    public static final LEDPattern ledOff = LEDPattern.solid(Color.kBlack);

    public static LEDPattern baseGreen = LEDPattern.solid(Color.kGreen);
    public static LEDPattern baseBlack = LEDPattern.solid(Color.kBlack);

    public static LEDPattern baseRed = LEDPattern.solid(Color.kRed);
    public static LEDPattern blinkingRed = baseRed.blink(Seconds.of(0.1));

    public static LEDPattern baseBlue = LEDPattern.solid(Color.kBlue);
    public static LEDPattern blinkingBlue = baseBlue.blink(Seconds.of(0.1));

    public static LEDPattern baseYellow = LEDPattern.solid(Color.kYellow);
    public static LEDPattern blinkingYellow = baseYellow.blink(Seconds.of(0.1));

    public static double speed = 0;
    public static final LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed,
        Color.kOrangeRed);
    public static LEDPattern mask = LEDPattern.progressMaskLayer(() -> speed);
    public static LEDPattern turboDisplay = base.mask(mask);

    public static boolean turboEffect = true;
  }
}
