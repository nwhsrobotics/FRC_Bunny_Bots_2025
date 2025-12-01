// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.util.RobotCANUtils.CANSparkFlexController;
import frc.robot.util.RobotCANUtils.CANSparkMaxController;
import frc.robot.util.RobotCANUtils.MotorKind;

public class ScoringSubsystem extends SubsystemBase { // Add FeedForward for the pivot

    public static enum States {
        Idle,

        Unload,

        LoadStageOne, // Spin intake and indexer wheels

        LoadStageTwo, // Spin only the intake wheels

        ShootHigh, // spin the flyWheels at high speed, feeder wheels and the indexer wheels

        ShootLow,// spin the flywheels at low speed
    }

    private States state = States.Idle;

    // shooter flywheels
    private final CANSparkFlexController flywheelMotor1;
    private final CANSparkFlexController flywheelMotor2;

    private final RelativeEncoder flywheelEncoder1;
    private final RelativeEncoder flywheelEncoder2;

    // feeder motor 
    private final CANSparkMaxController feederMotor;
    private final RelativeEncoder feederEncoder;

    // common SparkFlex config for both flywheels
    private final SparkFlexConfig flywheelCfg = new SparkFlexConfig();

    // pivot Motor declarations
    private CANSparkMaxController pivotMotor;
    private SparkMaxConfig pivotCfg = new SparkMaxConfig();
    private SparkClosedLoopController pivotPid;
    private double currentPos = 1.0; // starting pos
    private DutyCycleEncoder absEncoder;
    private RelativeEncoder motorEnc;
    private static final double ZERO_ROT = 0.803;

    public ScoringSubsystem() {

        
        absEncoder = new DutyCycleEncoder(0);
        pivotMotor = new CANSparkMaxController(
                2,
                MotorKind.NEO30AMP,
                pivotCfg,
                IdleMode.kBrake,
                0.1, 0.0, 0.0,
                1500, 1500, 0.6);
        pivotPid = pivotMotor.getClosedLoopController();
        motorEnc = pivotMotor.getEncoder();
        motorEnc.setPosition(Math.abs(absEncoder.get() - ZERO_ROT) * ScoringConstants.PIVOT_GEAR_RATIO);

        // ---------------- FLYWHEEL CONFIG ----------------
        // Configure PID + sensor on the SparkFlexConfig
        flywheelCfg.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0002)
                .i(0.0)
                .d(0.0);
        flywheelCfg.idleMode(IdleMode.kCoast);

        // create both flywheel motors using the wrapper constructor
        flywheelMotor1 = new CANSparkFlexController(
                Constants.CANAssignments.FLYWHEEL_ONE_ID,
                MotorKind.VORTEX,
                flywheelCfg,
                IdleMode.kCoast);

        flywheelMotor2 = new CANSparkFlexController(
                Constants.CANAssignments.FLYWHEEL_TWO_ID,
                MotorKind.VORTEX,
                flywheelCfg,
                IdleMode.kCoast);

        flywheelEncoder1 = flywheelMotor1.getEncoder();
        flywheelEncoder2 = flywheelMotor2.getEncoder();

        // ---------------- FEEDER SETUP ----------------
        feederMotor = new CANSparkMaxController(
                Constants.CANAssignments.FEEDER_MOTOR_ID,
                MotorKind.NEO550,
                new SparkMaxConfig(),
                IdleMode.kBrake,
                0.0003, 0, 0,
                2000, 2000, 10, 12.0);

        feederEncoder = feederMotor.getEncoder();
    }


    private boolean shooterAtSpeed() {
        double rpmOne = Math.abs(flywheelEncoder1.getVelocity());
        double rpmTwo = Math.abs(flywheelEncoder2.getVelocity());

        return Math.abs(rpmOne - Constants.CANAssignments.targetRPM) < Constants.CANAssignments.RPM_TOLERANCE
                && Math.abs(rpmTwo - Constants.CANAssignments.targetRPM) < Constants.CANAssignments.RPM_TOLERANCE;
    }

    private void setShooterRPM(double rpm) {
        Constants.CANAssignments.targetRPM = rpm;

        flywheelMotor1.getClosedLoopController().setReference(
                Constants.CANAssignments.targetRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                Constants.CANAssignments.targetRPM * Constants.CANAssignments.OUTPUT_PER_RPM * 12);

        flywheelMotor2.getClosedLoopController().setReference(
                Constants.CANAssignments.targetRPM,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                -Constants.CANAssignments.targetRPM * Constants.CANAssignments.OUTPUT_PER_RPM * 12);
    }

    private void stopShooter() {
        flywheelMotor1.set(0);
        flywheelMotor2.set(0);
        Constants.CANAssignments.targetRPM = 0;
    }

    // increase RPM and decrease RPM methods
    public void increaseRPM() {
        if (Constants.CANAssignments.targetRPM < 5500) {
            Constants.CANAssignments.targetRPM += 100;
        }
    }

    public void decreaseRPM() {
        if (Constants.CANAssignments.targetRPM > 100) {
            Constants.CANAssignments.targetRPM -= 100;
        }
    }

    private void runFeeder(double power) {
        feederMotor.set(power);
    }

    private void stopFeeder() {
        feederMotor.set(0);
    }


    public void intake() {
        currentPos = degreesToMotorRot(135); // CHANGE DEGREE TO BE MORE ACCURATE
        // TODO: add star wheel rotation logic
    }

    public void stow() {
        currentPos = degreesToMotorRot(0.8);
    }

    public double degreesToMotorRot(double degree) {
        return (degree / 360.0) * ScoringConstants.PIVOT_GEAR_RATIO;
    }

    public double getOffsetedCurrentPos() {
        double rot = absEncoder.get() - ZERO_ROT;
        return rot;
    }

    public void setState(States newState) {
        this.state = newState;
    }

    // ---------------- PERIODIC ----------------

    @Override
    public void periodic() {

        switch (state) {
            case Idle:
                stopShooter();
                stopFeeder();
                break;

            case LoadStageOne:
                // TODO: intake + feeder logic
                break;

            case LoadStageTwo:
                // TODO: intake-only logic
                break;

            case Unload:
                // TODO: reverse intake + feeder
                break;

            case ShootHigh:
                setShooterRPM(Constants.CANAssignments.HIGH_SHOT_RPM);
                stopFeeder();

                if (shooterAtSpeed()) {
                    runFeeder(+1.0);
                }
                break;

            case ShootLow:
                setShooterRPM(Constants.CANAssignments.LOW_SHOT_RPM);
                stopFeeder();

                if (shooterAtSpeed()) {
                    runFeeder(+1.0);
                }
                break;

            default:
                break;
        }

        pivotPid.setReference(currentPos, ControlType.kMAXMotionPositionControl);
        System.out.println(absEncoder.get() + "   " + motorEnc.getPosition() + "   " + currentPos);
    }

    public double getTargetRPM() {
        return Constants.CANAssignments.targetRPM;
    }

    public double getCurrentRPM() {
        return flywheelEncoder1.getVelocity();
    }

    public double getProgress() {
        double tgt = Constants.CANAssignments.targetRPM;
        if (tgt <= 0)
            return 0.0;
        double curr = Math.abs(getCurrentRPM());
        double p = curr / tgt;
        if (p < 0)
            return 0.0;
        if (p > 1)
            return 1.0;
        return p;
    }
}
