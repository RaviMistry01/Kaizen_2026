package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io = new ElevatorIORio();
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
 
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setPosition(double rotations) {
        ((ElevatorIORio) io).getLeft().getClosedLoopController()
                .setReference(rotations, SparkBase.ControlType.kMAXMotionPositionControl);

        Logger.recordOutput("Elevator/Setpoint", rotations);
    }

    public void manual(double power) {
        ((ElevatorIORio) io).getLeft().set(power);
        Logger.recordOutput("Elevator/ManualPower", power);
    }
}
