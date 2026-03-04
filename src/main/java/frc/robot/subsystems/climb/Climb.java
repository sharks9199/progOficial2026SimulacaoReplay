package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    
    private final ClimberIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private double currentPivotSetpoint = 0.0;
    private double currentWinchSetpoint = 0.0;

    public Climb(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
        Logger.recordOutput("Climb/PivotSetpoint", currentPivotSetpoint);
        Logger.recordOutput("Climb/WinchSetpoint", currentWinchSetpoint);
    }

    // ==========================================
    // MÉTODOS DO BRAÇO (PIVOT / CLAW)
    // ==========================================

    public double getPivotPosition() {
        return inputs.pivotPosition;
    }

    public void setPivotPosition(double setpoint) {
        currentPivotSetpoint = setpoint;
        io.setPivotSetpoint(setpoint);
    }

    public void setPivotSpeed(double speed) {
        io.setPivotSpeed(speed);
    }

    public void incrementPivotSetpoint(double delta) {
        setPivotPosition(currentPivotSetpoint + delta);
    }

    // ==========================================
    // MÉTODOS DA ESCALADA (WINCH / PLANETARY)
    // ==========================================

    public double getWinchPosition() {
        return inputs.winchPosition;
    }

    public void setWinchPosition(double setpoint) {
        currentWinchSetpoint = setpoint;
        io.setWinchSetpoint(setpoint);
    }

    public void setWinchSpeed(double speed) {
        io.setWinchSpeed(speed);
    }

    public void incrementWinchSetpoint(double delta) {
        setWinchPosition(currentWinchSetpoint + delta);
    }

    // ==========================================
    // MÉTODOS GERAIS
    // ==========================================

    public void stop() {
        io.setStopMode();
    }

    // ==========================================
    // COMMAND FACTORIES (Para usar no RobotContainer)
    // ==========================================

    public Command runPivotCommand(java.util.function.DoubleSupplier speedSupplier) {
        return this.run(() -> setPivotSpeed(speedSupplier.getAsDouble()));
    }

    public Command runWinchCommand(java.util.function.DoubleSupplier speedSupplier) {
        return this.run(() -> setWinchSpeed(speedSupplier.getAsDouble()));
    }

    public Command setPivotPositionCommand(double position) {
        return this.runOnce(() -> setPivotPosition(position));
    }

    public Command setWinchPositionCommand(double position) {
        return this.runOnce(() -> setWinchPosition(position));
    }
}