package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public static class ClimbIOInputs {
        public double pivotAbsolutePosition = 0.0;
        
        public double pivotPosition = 0.0; 
        public double pivotVelocity = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        public double winchPosition = 0.0;
        public double winchVelocity = 0.0;
        public double winchAppliedVolts = 0.0;
        public double winchCurrentAmps = 0.0;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default double getPivotPosition() { return 0.0; }
    public default double getWinchPosition() { return 0.0; }

    public default double getPivotSpeed() { return 0.0; }
    public default double getWinchSpeed() { return 0.0; }

    public default double getPivotSetpoint() { return 0.0; }
    public default double getWinchSetpoint() { return 0.0; }

    public default void setStopMode() {}

    public default void setPivotSpeed(double speed) {}
    public default void setWinchSpeed(double speed) {}

    public default void setPivotSetpoint(double setpoint) {}
    public default void setWinchSetpoint(double setpoint) {}
}