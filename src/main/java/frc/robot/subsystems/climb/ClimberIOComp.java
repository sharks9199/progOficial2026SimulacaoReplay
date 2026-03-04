package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOComp implements ClimberIO {

    private final SparkMax planetaryMotor;
    private final SparkMax clawMotor;

    private final SparkClosedLoopController planetaryPID;
    private final SparkClosedLoopController clawPID;

    private double currentPlanetarySetpoint = 0.0;
    private double currentClawSetpoint = 0.0;

    public ClimberIOComp() {
        planetaryMotor = new SparkMax(ClimbConstants.planetaryMotorID, MotorType.kBrushless);
        clawMotor = new SparkMax(ClimbConstants.clawMotorID, MotorType.kBrushless);

        SparkMaxConfig planetaryMotorConfig = new SparkMaxConfig();
        SparkMaxConfig clawMotorConfig = new SparkMaxConfig();

        planetaryMotorConfig
                .smartCurrentLimit(60)
                .idleMode(IdleMode.kBrake)
                .inverted(true);

        planetaryMotorConfig.closedLoop
                .pid(ClimbConstants.climbP, 0.0, 0.0);

        clawMotorConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)
                .inverted(false);

        clawMotorConfig.closedLoop
                .pid(ClimbConstants.clawP, 0.0, 0.0);

        planetaryMotor.configure(planetaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clawMotor.configure(clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        planetaryPID = planetaryMotor.getClosedLoopController();
        clawPID = clawMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.pivotPosition = clawMotor.getEncoder().getPosition();
        inputs.pivotVelocity = clawMotor.getEncoder().getVelocity();
        inputs.pivotAppliedVolts = clawMotor.getAppliedOutput() * clawMotor.getBusVoltage();
        inputs.pivotCurrentAmps = clawMotor.getOutputCurrent();

        inputs.winchPosition = planetaryMotor.getEncoder().getPosition();
        inputs.winchVelocity = planetaryMotor.getEncoder().getVelocity();
        inputs.winchAppliedVolts = planetaryMotor.getAppliedOutput() * planetaryMotor.getBusVoltage();
        inputs.winchCurrentAmps = planetaryMotor.getOutputCurrent();
    }

    @Override
    public double getPivotPosition() {
        return clawMotor.getEncoder().getPosition();
    }

    @Override
    public double getWinchPosition() {
        return planetaryMotor.getEncoder().getPosition();
    }

    @Override
    public double getPivotSpeed() {
        return clawMotor.getEncoder().getVelocity();
    }

    @Override
    public double getWinchSpeed() {
        return planetaryMotor.getEncoder().getVelocity();
    }

    @Override
    public double getPivotSetpoint() {
        return currentClawSetpoint;
    }

    @Override
    public double getWinchSetpoint() {
        return currentPlanetarySetpoint;
    }

    @Override
    public void setStopMode() {
        clawMotor.set(0.0);
        planetaryMotor.set(0.0);
    }

    @Override
    public void setPivotSpeed(double speed) {
        clawMotor.set(speed);
    }

    @Override
    public void setWinchSpeed(double speed) {
        planetaryMotor.set(speed);
    }

    @Override
    public void setPivotSetpoint(double setpoint) {
        currentClawSetpoint = setpoint;
        clawPID.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void setWinchSetpoint(double setpoint) {
        currentPlanetarySetpoint = setpoint;
        planetaryPID.setReference(setpoint, ControlType.kPosition);
    }
}