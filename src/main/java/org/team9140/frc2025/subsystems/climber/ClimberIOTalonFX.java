package org.team9140.frc2025.subsystems.climber;

import static org.team9140.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX rightMotor = new TalonFX(Constants.Ports.CLIMBER_MOTOR_RIGHT, "sigma");
    private final TalonFX leftMotor = new TalonFX(Constants.Ports.CLIMBER_MOTOR_LEFT, "sigma");

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<Voltage> followerAppliedVolts;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Current> followerSupplyCurrent;
    private final StatusSignal<Temperature> followerTemp;

    private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

    public ClimberIOTalonFX() {
        this.leftMotor.setControl(new Follower(this.rightMotor.getDeviceID(), true));

        CurrentLimitsConfigs currentLimitsConfigs =
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Constants.Climber.STATOR_LIMIT)
                        .withStatorCurrentLimitEnable(true);

        MotorOutputConfigs motorOutputConfigs =
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake);

        FeedbackConfigs feedbackConfigs =
                new FeedbackConfigs().withSensorToMechanismRatio(Constants.Climber.GEAR_RATIO);

        SoftwareLimitSwitchConfigs softLimits =
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(Constants.Climber.SOFT_LIMIT_HIGHER)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(Constants.Climber.SOFT_LIMIT_LOWER)
                        .withReverseSoftLimitEnable(true);

        TalonFXConfiguration motorConfig =
                new TalonFXConfiguration()
                        .withCurrentLimits(currentLimitsConfigs)
                        .withFeedback(feedbackConfigs)
                        .withMotorOutput(motorOutputConfigs)
                        .withSoftwareLimitSwitch(softLimits);

        tryUntilOk(
                5,
                () ->
                        rightMotor
                                .getConfigurator()
                                .apply(
                                        motorConfig
                                                .withCurrentLimits(currentLimitsConfigs)
                                                .withFeedback(feedbackConfigs)
                                                .withMotorOutput(motorOutputConfigs)
                                                .withSoftwareLimitSwitch(softLimits),
                                        0.25));

        position = this.rightMotor.getPosition();
        velocity = this.rightMotor.getVelocity();
        appliedVolts = this.rightMotor.getMotorVoltage();
        torqueCurrent = this.rightMotor.getTorqueCurrent();
        supplyCurrent = this.rightMotor.getSupplyCurrent();
        temp = this.rightMotor.getDeviceTemp();
        followerAppliedVolts = this.leftMotor.getMotorVoltage();
        followerTorqueCurrent = this.leftMotor.getTorqueCurrent();
        followerSupplyCurrent = this.leftMotor.getSupplyCurrent();
        followerTemp = this.leftMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVolts,
                supplyCurrent,
                temp,
                followerAppliedVolts,
                followerTorqueCurrent,
                followerSupplyCurrent,
                followerTemp);
        torqueCurrent.setUpdateFrequency(250);
        ParentDevice.optimizeBusUtilizationForAll(this.rightMotor, this.leftMotor);

        PhoenixUtil.registerSignals(
                true,
                position,
                velocity,
                appliedVolts,
                torqueCurrent,
                supplyCurrent,
                temp,
                followerAppliedVolts,
                followerTorqueCurrent,
                followerSupplyCurrent,
                followerTemp);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            runVolts(0.0);
        }

        inputs.data =
                new ClimberIOData(
                        // Exclude torque-current b/c it's running at a much higher update rate
                        BaseStatusSignal.isAllGood(
                                position, velocity, appliedVolts, supplyCurrent, temp),
                        BaseStatusSignal.isAllGood(
                                followerAppliedVolts,
                                followerTorqueCurrent,
                                followerSupplyCurrent,
                                followerTemp),
                        Units.rotationsToRadians(position.getValueAsDouble()),
                        Units.rotationsToRadians(velocity.getValueAsDouble()),
                        appliedVolts.getValueAsDouble(),
                        torqueCurrent.getValueAsDouble(),
                        supplyCurrent.getValueAsDouble(),
                        temp.getValueAsDouble(),
                        followerAppliedVolts.getValueAsDouble(),
                        followerTorqueCurrent.getValueAsDouble(),
                        followerSupplyCurrent.getValueAsDouble(),
                        followerTemp.getValueAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        this.rightMotor.setControl(this.voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        this.rightMotor.stopMotor();
    }
}
