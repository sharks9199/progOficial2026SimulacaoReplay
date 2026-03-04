
package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    private final AddressableLEDBufferView allianceView;
    private final AddressableLEDBufferView shooterView;
    private final AddressableLEDBufferView boostView;
    private final AddressableLEDBufferView shooterViewSecondary;
    private final AddressableLEDBufferView boostViewSecondary;

    private BooleanSupplier isShooterAligned;
    private BooleanSupplier hasTarget;
    private DoubleSupplier currentSpeed;


    public LEDSubsystem(BooleanSupplier isShooterAligned, BooleanSupplier hasTarget, DoubleSupplier currentSpeed) {
        this.isShooterAligned = isShooterAligned;
        this.hasTarget = hasTarget;
        this.currentSpeed = currentSpeed;

        m_led = new AddressableLED(LEDConstants.kLEDPort);
        m_buffer = new AddressableLEDBuffer(LEDConstants.kTotalLength);
        m_led.setLength(m_buffer.getLength());

        allianceView = m_buffer.createView(LEDConstants.kAllianceStart, LEDConstants.kAllianceEnd);
        shooterView = m_buffer.createView(LEDConstants.kShooterStart, LEDConstants.kShooterEnd);
        boostView = m_buffer.createView(LEDConstants.kBoostStart, LEDConstants.kBoostEnd);
        shooterViewSecondary = m_buffer.createView(LEDConstants.kShooterStartSecondary, LEDConstants.kShooterEndSecondary);
        boostViewSecondary = m_buffer.createView(LEDConstants.kBoostStartSecondary, LEDConstants.kBoostEndSecondary);

        m_led.setData(m_buffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        updateAllianceColor();
        updateShooterStatus();
        updateSpeedBar();

        m_led.setData(m_buffer);
    }

    private void updateAllianceColor() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        Color color = Color.kBlack;

        if (alliance.isPresent()) {
            color = alliance.get() == Alliance.Red ? Color.kDarkRed : Color.kDarkBlue;
        }
        else {
            color = Color.kMaroon; // Default color if alliance is unknown
        }

        for (int i = 0; i < allianceView.getLength(); i++) {
            allianceView.setLED(i, color);
        }
        
    }

    private void updateShooterStatus() {
        boolean target = hasTarget.getAsBoolean();
        boolean aligned = isShooterAligned.getAsBoolean();
        Color statusColor;

        if (aligned && target) {
            statusColor = Color.kDarkGreen;
        } else if (target && !aligned) {
            statusColor = Color.kPurple;
        } else {
            double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            statusColor = (timestamp % 0.5 < 0.25) ? Color.kRed : Color.kBlack;
        }

        for (int i = 0; i < shooterView.getLength(); i++) {
            shooterView.setLED(i, statusColor);
        }
        for (int i = 0; i < shooterViewSecondary.getLength(); i++) {
            shooterViewSecondary.setLED(i, statusColor);
        }
    }

    private void updateSpeedBar() {
        double maxSpeed = 4.5;
        double speed = Math.abs(currentSpeed.getAsDouble());
        int ledsLit = (int) ((speed / maxSpeed) * boostView.getLength());
        ledsLit = Math.max(0, Math.min(ledsLit, boostView.getLength()));
        Color boostColor = Color.kOrangeRed;
        Color baseColor = Color.kBlack;

        for (int i = 0; i < boostView.getLength(); i++) {
            if (i < ledsLit) {
                boostView.setLED(i, boostColor);
            } else {
                boostView.setLED(i, baseColor);
            }
        }
         for (int i = 0; i < boostViewSecondary.getLength(); i++) {
            if (i < ledsLit) {
                boostViewSecondary.setLED(i, boostColor);
            } else {
                boostViewSecondary.setLED(i, baseColor);
            }
        }
    }
}
