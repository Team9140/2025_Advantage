package org.team9140.lib.rollers;

// TODO: Move PhoenixUtil to lib
import static org.team9140.frc2025.util.PhoenixUtil.registerSignals;
import static org.team9140.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerIOTalonFX implements RollerIO {
    private final TalonFX talon;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;
    private final StatusSignal<Boolean> tempFault;

    // Single shot for voltage mode, robot loop will call continuously
    private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final NeutralOut neutralOut = new NeutralOut();

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public RollerIOTalonFX(
            int id, String bus, double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {
        talon = new TalonFX(id, bus);

        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Feedback.VelocityFilterTimeConstant = 0.1;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        tryUntilOk(5, () -> talon.getConfigurator().apply(config));

        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();
        tempFault = talon.getFault_DeviceTemp();

        tryUntilOk(
                5,
                () ->
                        BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0,
                                appliedVoltage,
                                supplyCurrent,
                                torqueCurrent,
                                tempCelsius,
                                tempFault));
        tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));

        // Register signals for refresh
        registerSignals(
                new CANBus(bus).isNetworkFD(),
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                tempFault);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.data =
                new RollerIOData(
                        this.appliedVoltage.getValueAsDouble(),
                        this.supplyCurrent.getValueAsDouble(),
                        this.torqueCurrent.getValueAsDouble(),
                        tempCelsius.getValueAsDouble(),
                        tempFault.getValue(),
                        BaseStatusSignal.isAllGood(
                                appliedVoltage,
                                supplyCurrent,
                                torqueCurrent,
                                tempCelsius,
                                tempFault));
    }

    @Override
    public void runVolts(double volts) {
        this.talon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stop() {
        this.talon.setControl(neutralOut);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        this.config.MotorOutput.NeutralMode =
                brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    }

    @Override
    public void setInverted(boolean inverted) {
        config.MotorOutput.Inverted =
                inverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    }
}
