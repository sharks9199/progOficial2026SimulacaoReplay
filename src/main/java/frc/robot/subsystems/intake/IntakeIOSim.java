package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId; 
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeConstants.intakeConstants;

public class IntakeIOSim implements IntakeIO {

    private static final double WHEEL_GEARING = 0.5;
    private static final double WHEEL_MOI = 0.005; 

    private static final double GEAR_RATIO = 40.0;
    private static final double ARM_LENGTH = 0.4;
    private static final double ARM_MASS = 2.0;

    // --- A ÚNICA MUDANÇA: Abrimos muito os limites para o braço não bater na parede virtual e travar ---
    private static final double MIN_ANGLE = Units.degreesToRadians(-360);
    private static final double MAX_ANGLE = Units.degreesToRadians(360);
    // ---------------------------------------------------------------------------------------------------

    private final SingleJointedArmSim rotationSim;
    private final FlywheelSim wheelSim;

    // PID Simulado e Controle de Estado
    private final PIDController simPid = new PIDController(24.0, 0.0, 0.0);
    private boolean isClosedLoop = true; 

    private double rotationAppliedVolts = 0.0;
    private double wheelAppliedVolts = 0.0;

    public IntakeIOSim() {
        rotationSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            GEAR_RATIO, 
            SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS), 
            ARM_LENGTH, 
            MIN_ANGLE, 
            MAX_ANGLE, 
            true, 
            0.0   
        );

        wheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1), 
                WHEEL_MOI, 
                WHEEL_GEARING
            ),
            DCMotor.getKrakenX60(1),
            WHEEL_GEARING
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        
        // Lógica que simula o PID interno do motor real
        if (isClosedLoop) {
            double currentPosition = Units.radiansToRotations(rotationSim.getAngleRads());
            double pidVolts = simPid.calculate(currentPosition, intakeConstants.intakeSetpoint);
            rotationAppliedVolts = MathUtil.clamp(pidVolts, -12.0, 12.0);
            rotationSim.setInputVoltage(rotationAppliedVolts);
        }

        rotationSim.update(0.020);
        wheelSim.update(0.020);

        inputs.rotationPosition = Units.radiansToRotations(rotationSim.getAngleRads()*GEAR_RATIO);
        inputs.rotationVelocity = Units.radiansToRotations(rotationSim.getVelocityRadPerSec());
        inputs.rotationAppliedVolts = rotationAppliedVolts;
        inputs.rotationCurrentAmps = rotationSim.getCurrentDrawAmps();

        inputs.wheelVelocity = Units.radiansToRotations(wheelSim.getAngularVelocityRadPerSec());
        inputs.wheelAppliedVolts = wheelAppliedVolts;
        inputs.wheelCurrentAmps = wheelSim.getCurrentDrawAmps();
    }

    @Override
    public double getPosition() {
        return Units.radiansToRotations(rotationSim.getAngleRads())* GEAR_RATIO;
    }

    @Override
    public double getSpeed() {
        return Units.radiansToRotations(rotationSim.getVelocityRadPerSec())* GEAR_RATIO;
    }

    @Override
    public double getSetpoint() {
        return intakeConstants.intakeSetpoint;
    }

    @Override
    public void setStopMode() {
        isClosedLoop = false; 
        rotationAppliedVolts = 0.0;
        rotationSim.setInputVoltage(0.0);
    }

    @Override
    public void setPlanetary(double speed) {
        isClosedLoop = false; 
        speed = MathUtil.clamp(speed, -intakeConstants.IntakeMaxSpeed, intakeConstants.IntakeMaxSpeed);
        rotationAppliedVolts = speed * 12.0;
        rotationSim.setInputVoltage(rotationAppliedVolts);
    }

    @Override
    public void setIntake(double speed) {
        wheelAppliedVolts = speed * 12.0;
        wheelSim.setInputVoltage(wheelAppliedVolts);
    }

    @Override
    public void changeSetpoint(double setpoint) {
        isClosedLoop = true; 
        intakeConstants.intakeSetpoint = setpoint;
    }
}