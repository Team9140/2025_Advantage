// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.team9140.frc2025.subsystems.superstructure.elevator;

import static org.team9140.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Constants.Ports;
import org.team9140.frc2025.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX rightMotor;
    private final TalonFX leftMotor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

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

    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0)
            .withUpdateFreqHz(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

    public ElevatorIOTalonFX() {
        this.rightMotor = new TalonFX(Ports.ELEVATOR_MOTOR_RIGHT, "sigma");
        this.leftMotor = new TalonFX(Ports.ELEVATOR_MOTOR_LEFT, "sigma");
        this.leftMotor.setControl(new Follower(this.rightMotor.getDeviceID(), true));

        Slot0Configs elevatorGains = new Slot0Configs()
                .withKP(Constants.Elevator.kP)
                .withKI(Constants.Elevator.kI)
                .withKD(Constants.Elevator.kD);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Elevator.STATOR_LIMIT)
                .withStatorCurrentLimitEnable(true);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Elevator.GEAR_RATIO);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(
                        Constants.Elevator.SOFT_LIMIT
                                .div(Constants.Elevator.SPOOL_CIRCUMFERENCE)
                                .magnitude())
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0.0);

        tryUntilOk(
                5,
                () -> rightMotor
                        .getConfigurator()
                        .apply(
                                config
                                        .withSlot0(elevatorGains)
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

        // Register signals for refresh
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
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.data = new ElevatorIOData(
                // Exclude torque-current b/c it's running at a much higher update rate
                BaseStatusSignal.isAllGood(position, velocity, appliedVolts, supplyCurrent, temp),
                BaseStatusSignal.isAllGood(
                        followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp),
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
    public void runOpenLoop(double output) {
        this.rightMotor.setControl(torqueCurrentRequest.withOutput(output));
    }

    @Override
    public void runVolts(double volts) {
        this.rightMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        this.rightMotor.stopMotor();
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {
        this.rightMotor.setControl(
                positionTorqueCurrentRequest
                        .withPosition(Units.radiansToRotations(positionRad))
                        .withFeedForward(feedforward));
    }

    // Probably not needed cuz we don't use tuneable numbers or anything
    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        tryUntilOk(5, () -> this.rightMotor.getConfigurator().apply(config));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(
                () -> this.rightMotor.setNeutralMode(
                        enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
                .start();
    }
}
