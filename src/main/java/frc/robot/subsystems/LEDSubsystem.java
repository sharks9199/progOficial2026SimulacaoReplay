package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED m_led;
    private final AddressableLEDBuffer ledBuffer;

    // Suppliers mais limpos
    private final BooleanSupplier m_readyToShootSupplier;
    private final DoubleSupplier m_robotVelocitySupplier;

    private double m_brilho = 1.0;

    private final int LED_LENGTH = 216; 

    private final int BAR1_START = 18;
    private final int BAR1_END = 41;
    private final int BAR2_START = 177;
    private final int BAR2_END = 200;

    private final double MAX_VELOCITY_MPS = 5.0;

    public LEDSubsystem(BooleanSupplier readyToShootSupplier, DoubleSupplier robotVelocitySupplier) {
        this.m_readyToShootSupplier = readyToShootSupplier;
        this.m_robotVelocitySupplier = robotVelocitySupplier;

        m_led = new AddressableLED(9); 
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        m_led.setLength(ledBuffer.getLength());

        SmartDashboard.putNumber("LED Brilho", 1.0);

        limparBuffer();
        m_led.start();
    }

    private void limparBuffer() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    private void atualizarEstadoAtirador() {
        boolean readyToShoot = m_readyToShootSupplier.getAsBoolean();
        
        int r = readyToShoot ? 0 : (int)(255 * m_brilho);
        int g = readyToShoot ? (int)(255 * m_brilho) : 0;
        int b = 0;

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (!isInVelocityBar(i)) {
                ledBuffer.setRGB(i, r, g, b);
            }
        }
    }

    private void atualizarBarrasVelocidade() {
        double velocity = m_robotVelocitySupplier.getAsDouble();
        double percentage = Math.max(0.0, Math.min(1.0, velocity / MAX_VELOCITY_MPS));

        int r = (int)(255 * m_brilho);
        int g = (int)(165 * m_brilho);
        int b = 0;

        // Barra 1: Lado esquerdo (Sobe do 18 ao 41) - reverse = false
        applyBarGradient(BAR1_START, BAR1_END, percentage, r, g, b, false);

        // Barra 2: Lado direito (Sobe do 200 ao 177) - reverse = true
        applyBarGradient(BAR2_START, BAR2_END, percentage, r, g, b, true);
    }

    private void applyBarGradient(int startIdx, int endIdx, double percentage, int r, int g, int b, boolean reverse) {
        int length = endIdx - startIdx + 1;
        int ledsAcesos = (int) Math.round(length * percentage);

        for (int i = 0; i < length; i++) {
            int ledIndex = reverse ? (endIdx - i) : (startIdx + i);
            
            if (i < ledsAcesos) {
                ledBuffer.setRGB(ledIndex, r, g, b);
            } else {
                ledBuffer.setRGB(ledIndex, 0, 0, 0);
            }
        }
    }

    private boolean isInVelocityBar(int index) {
        return (index >= BAR1_START && index <= BAR1_END) || 
               (index >= BAR2_START && index <= BAR2_END);
    }

    @Override
    public void periodic() {
        double valorLido = SmartDashboard.getNumber("LED Brilho", 1.0);
        m_brilho = Math.max(0.0, Math.min(1.0, valorLido));

        atualizarEstadoAtirador();
        atualizarBarrasVelocidade();

        m_led.setData(ledBuffer);
    }
}