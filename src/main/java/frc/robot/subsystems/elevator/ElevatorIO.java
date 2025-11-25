package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double leftPosition = 0.0;
        public double rightPosition = 0.0;

        public double leftVelocity = 0.0;
        public double rightVelocity = 0.0;

        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;

        public boolean bottomLimit = false;
        public boolean topLimit = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }
}
