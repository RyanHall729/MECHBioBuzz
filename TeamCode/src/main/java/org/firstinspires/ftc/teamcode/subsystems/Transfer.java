package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Locale;

/**
 * The Transfer subsystem manages two motors (transferMotor1 and transferMotor2)
 * responsible for transferring game elements within the robot.
 */
public class Transfer {

    // --- Enum for Transfer States ---
    public enum TransferState {
        OFF,
        FORWARD,
        REVERSE
    }

    // --- Hardware ---
    private final DcMotorEx transferMotor1;
    private final DcMotorEx transferMotor2;

    // --- State & Power Variables ---
    private TransferState currentTransferState = TransferState.OFF;
    private TransferState previousTransferState = TransferState.OFF;

    private double forwardPower = 1.0;
    private double reversePower = -1.0;

    private double lastTransferMotor1Power = 0.0;
    private double lastTransferMotor2Power = 0.0;

    /**
     * Initializes the Transfer subsystem using the centralized RobotHardware hub.
     *
     * @param robot Central hardware configuration object.
     */
    public Transfer(RobotHardware robot) {
        this(robot != null ? robot.transferMotor1 : null, robot != null ? robot.transferMotor2 : null);
    }

    /**
     * Initializes the Transfer subsystem with direct motor instances.
     *
     * @param transferMotor1 First transfer motor.
     * @param transferMotor2 Second transfer motor.
     */
    public Transfer(DcMotorEx transferMotor1, DcMotorEx transferMotor2) {
        this.transferMotor1 = transferMotor1;
        this.transferMotor2 = transferMotor2;

        configureMotors();
    }

    /**
     * Configures motor directions, zero-power behaviors, and run modes.
     * Left motor (transferMotor1): FORWARD (Clockwise)
     * Right motor (transferMotor2): REVERSE (Counter-Clockwise)
     */
    private void configureMotors() {
        if (transferMotor1 != null) {
            // Left motor spins Clockwise (FORWARD)
            transferMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
            transferMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transferMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferMotor1.setPower(0);
        }

        if (transferMotor2 != null) {
            // Right motor spins Counter-Clockwise (REVERSE)
            transferMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            transferMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transferMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferMotor2.setPower(0);
        }
    }

    /**
     * Updates the state machine and sets motor powers continuously.
     * Should be called in every loop iteration of an OpMode.
     */
    public void update() {
        if (currentTransferState != previousTransferState) {
            switch (currentTransferState) {
                case FORWARD:
                    applyPower(forwardPower, forwardPower);
                    break;
                case REVERSE:
                    applyPower(reversePower, reversePower);
                    break;
                case OFF:
                default:
                    applyPower(0.0, 0.0);
                    break;
            }
            previousTransferState = currentTransferState;
        }
    }

    /**
     * Internal helper to set motor powers safely with null-checks.
     */
    private void applyPower(double p1, double p2) {
        this.lastTransferMotor1Power = p1;
        this.lastTransferMotor2Power = p2;

        if (transferMotor1 != null) {
            transferMotor1.setPower(p1);
        }
        if (transferMotor2 != null) {
            transferMotor2.setPower(p2);
        }
    }

    /**
     * Changes the subsystem state.
     *
     * @param newState Desired TransferState.
     */
    public void changeState(TransferState newState) {
        if (newState != null && newState != currentTransferState) {
            System.out.printf(Locale.US, "TRANSFER: Previous state: %s New State: %s%n", currentTransferState, newState);
            currentTransferState = newState;
        }
    }

    /**
     * Sets state to FORWARD.
     */
    public void forward() {
        changeState(TransferState.FORWARD);
    }

    /**
     * Sets state to REVERSE.
     */
    public void reverse() {
        changeState(TransferState.REVERSE);
    }

    /**
     * Stops the transfer motors and resets state to OFF.
     */
    public void stop() {
        changeState(TransferState.OFF);
        applyPower(0.0, 0.0);
    }

    /**
     * Directly sets manual power for both motors, switching state to OFF or custom state.
     *
     * @param power Target power (-1.0 to 1.0).
     */
    public void setPower(double power) {
        setPower(power, power);
    }

    /**
     * Directly sets manual power for transferMotor1 and transferMotor2 independently.
     *
     * @param transferMotor1Power Power for transferMotor1.
     * @param transferMotor2Power Power for transferMotor2.
     */
    public void setPower(double transferMotor1Power, double transferMotor2Power) {
        applyPower(transferMotor1Power, transferMotor2Power);
    }

    /**
     * Sets motor directions for transferMotor1 and transferMotor2.
     *
     * @param dir1 Direction for transferMotor1.
     * @param dir2 Direction for transferMotor2.
     */
    public void setMotorDirections(DcMotorSimple.Direction dir1, DcMotorSimple.Direction dir2) {
        if (transferMotor1 != null) {
            transferMotor1.setDirection(dir1);
        }
        if (transferMotor2 != null) {
            transferMotor2.setDirection(dir2);
        }
    }

    /**
     * Sets the ZeroPowerBehavior for both transfer motors (BRAKE or FLOAT).
     *
     * @param behavior Desired ZeroPowerBehavior.
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (transferMotor1 != null) {
            transferMotor1.setZeroPowerBehavior(behavior);
        }
        if (transferMotor2 != null) {
            transferMotor2.setZeroPowerBehavior(behavior);
        }
    }

    /**
     * Sets the RunMode for both transfer motors.
     *
     * @param runMode Desired RunMode.
     */
    public void setRunMode(DcMotor.RunMode runMode) {
        if (transferMotor1 != null) {
            transferMotor1.setMode(runMode);
        }
        if (transferMotor2 != null) {
            transferMotor2.setMode(runMode);
        }
    }

    /**
     * Resets encoder counts for both transfer motors.
     */
    public void resetEncoders() {
        if (transferMotor1 != null) {
            transferMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            transferMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (transferMotor2 != null) {
            transferMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            transferMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // --- Getters & Setters ---

    public TransferState getTransferState() {
        return currentTransferState;
    }

    public double getForwardPower() {
        return forwardPower;
    }

    public void setForwardPower(double forwardPower) {
        this.forwardPower = forwardPower;
    }

    public double getReversePower() {
        return reversePower;
    }

    public void setReversePower(double reversePower) {
        this.reversePower = reversePower;
    }

    public double getTransferMotor1Power() {
        return lastTransferMotor1Power;
    }

    public double getTransferMotor2Power() {
        return lastTransferMotor2Power;
    }

    public int getTransferMotor1Position() {
        return transferMotor1 != null ? transferMotor1.getCurrentPosition() : 0;
    }

    public int getTransferMotor2Position() {
        return transferMotor2 != null ? transferMotor2.getCurrentPosition() : 0;
    }

    public DcMotorEx getTransferMotor1() {
        return transferMotor1;
    }

    public DcMotorEx getTransferMotor2() {
        return transferMotor2;
    }

    public DcMotorEx getLeftMotor() {
        return transferMotor1;
    }

    public DcMotorEx getRightMotor() {
        return transferMotor2;
    }

    /**
     * Telemetry helper method to display subsystem status.
     *
     * @param telemetry Telemetry object from OpMode.
     */
    public void addTelemetry(Telemetry telemetry) {
        if (telemetry != null) {
            telemetry.addData("Transfer State", currentTransferState.toString());
            telemetry.addData("Transfer Motor 1 Power", "%.2f", lastTransferMotor1Power);
            telemetry.addData("Transfer Motor 2 Power", "%.2f", lastTransferMotor2Power);
            telemetry.addData("Transfer Motor 1 Pos", getTransferMotor1Position());
            telemetry.addData("Transfer Motor 2 Pos", getTransferMotor2Position());
        }
    }
}
