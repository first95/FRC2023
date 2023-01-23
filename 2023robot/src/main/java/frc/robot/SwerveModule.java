package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.BetterSwerveModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Drivebase;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset, lastAngle;
    private CANSparkMax angleMotor, driveMotor;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder angleEncoder, driveEncoder;
    private SparkMaxPIDController angleController, driveController;
    private double angle, omega, speed, fakePos, lastTime;
    private Timer time;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        angle = 0;
        speed = 0;
        fakePos = 0;
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        driveMotor.restoreFactoryDefaults();

        // Config angle encoders
        absoluteEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);
        absoluteEncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION / 60);
        absoluteEncoder.setZeroOffset(angleOffset);
        absoluteEncoder.setInverted(Drivebase.ABSOLUTE_ENCODER_INVERT);

        // Config angle motor/controller
        angleController = angleMotor.getPIDController();
        angleController.setP(Drivebase.MODULE_KP);
        angleController.setI(Drivebase.MODULE_KI);
        angleController.setD(Drivebase.MODULE_KD);
        angleController.setFF(Drivebase.MODULE_KF);
        angleController.setIZone(Drivebase.MODULE_IZ);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(180);
        angleController.setPositionPIDWrappingMinInput(-180);
        angleController.setFeedbackDevice(absoluteEncoder);
        angleMotor.setInverted(Drivebase.ANGLE_MOTOR_INVERT);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // Config drive motor/controller
        driveController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);
        driveEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION / 60);
        driveController.setP(Drivebase.VELOCITY_KP);
        driveController.setI(Drivebase.VELOCITY_KI);
        driveController.setD(Drivebase.VELOCITY_KD);
        driveController.setFF(Drivebase.VELOCITY_KF);
        driveController.setIZone(Drivebase.VELOCITY_IZ);
        driveMotor.setInverted(Drivebase.DRIVE_MOTOR_INVERT);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveMotor.burnFlash();
        angleMotor.burnFlash();

        lastAngle = getState().angle.getDegrees();

        if (!Robot.isReal()) {
            time = new Timer();
            time.start();
            lastTime = time.get();
        }
    }

    public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState simpleState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
        desiredState = new BetterSwerveModuleState(simpleState.speedMetersPerSecond, simpleState.angle, desiredState.omegaRadPerSecond);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Drivebase.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond;
            driveController.setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(velocity));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Drivebase.MAX_SPEED * 0.01) ? 
            lastAngle :
            desiredState.angle.getDegrees()); // Prevents module rotation if speed is less than 1%
        angleController.setReference(angle, ControlType.kPosition, 0, Math.toDegrees(desiredState.omegaRadPerSecond) * Drivebase.MODULE_KV);
        lastAngle = angle;

        this.angle = desiredState.angle.getDegrees();
        omega = desiredState.omegaRadPerSecond;
        speed = desiredState.speedMetersPerSecond;

        if (!Robot.isReal()) {
            double dt = time.get() - lastTime;
            fakePos += (speed * dt);
            lastTime = time.get();
        }
    }

    public BetterSwerveModuleState getState() {
        double velocity;
        Rotation2d azimuth;
        double omega;
        if (Robot.isReal()) {
            velocity = driveEncoder.getVelocity();
            azimuth = Rotation2d.fromDegrees(angleEncoder.getPosition());
            omega = angleEncoder.getVelocity();
        } else {
            velocity = speed;
            azimuth = Rotation2d.fromDegrees(this.angle);
            omega = this.omega;
        }
        return new BetterSwerveModuleState(velocity, azimuth, omega);
    }

    public SwerveModulePosition getPosition() {
        double position;
        Rotation2d azimuth;
        if (Robot.isReal()) {
            position = driveEncoder.getPosition();
            azimuth = Rotation2d.fromDegrees(angleEncoder.getPosition());
        } else {
            position = fakePos;
            azimuth = Rotation2d.fromDegrees(angle);
        }
        return new SwerveModulePosition(position, azimuth);
    }

    public double getAbsoluteEncoder() {
        return angleEncoder.getPosition();
    }

    public void setMotorBrake(boolean brake) {
        driveMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    public void turnModule(double speed) {
        angleMotor.set(speed);
        SmartDashboard.putNumber("AbsoluteEncoder" + moduleNumber, absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("ControlEffort" + moduleNumber, angleMotor.getAppliedOutput());
    }
}
