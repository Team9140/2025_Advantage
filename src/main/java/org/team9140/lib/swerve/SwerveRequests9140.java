package org.team9140.lib.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.Current;

public class SwerveRequests9140 {
    public static class SysIdSwerveSteerTorqueCurrentFOC implements SwerveRequest {

        public Current ampsToApply = Amps.of(0.0);

        private final CoastOut m_driveRequest = new CoastOut();

        private final TorqueCurrentFOC m_steerRequest = new TorqueCurrentFOC(0.0);

        @Override
        public StatusCode apply(
                SwerveControlParameters parameters,
                @SuppressWarnings("rawtypes") SwerveModule... modulesToApply) {
            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(m_driveRequest, m_steerRequest.withOutput(ampsToApply));
            }
            return StatusCode.OK;
        }

        /**
         * Sets the current to apply to the steer motors.
         *
         * @param amps Current to apply, wpilib needs to think is a voltage
         * @return this request
         */
        public SysIdSwerveSteerTorqueCurrentFOC withAmps(double amps) {
            ampsToApply = Amps.of(amps);
            return this;
        }
        /**
         * Sets the current to apply to the steer motors.
         *
         * @param volts Current to apply, wpilib needs to think is a voltage
         * @return this request
         */
        public SysIdSwerveSteerTorqueCurrentFOC withAmps(Current amps) {
            ampsToApply = amps;
            return this;
        }
    }
}
