package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorIORio implements ElevatorIO {

    private final SparkFlex left = new SparkFlex(8, MotorType.kBrushless);
    private final SparkFlex right = new SparkFlex(9, MotorType.kBrushless);

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftPosition = left.getEncoder().getPosition();
        inputs.rightPosition = right.getEncoder().getPosition();

        inputs.leftVelocity = left.getEncoder().getVelocity();
        inputs.rightVelocity = right.getEncoder().getVelocity();

        inputs.leftCurrent = left.getOutputCurrent();
        inputs.rightCurrent = right.getOutputCurrent();
    }

    public SparkFlex getLeft() { return left; }
    public SparkFlex getRight() { return right; }
}
