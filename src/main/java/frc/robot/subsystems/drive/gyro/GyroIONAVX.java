package frc.robot.subsystems.drive.gyro;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveSubsystemConstants;

public class GyroIONAVX implements GyroIO {
    private static AHRS m_gyro;

    public GyroIONAVX() {
        m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    }

    @Override
    public double getGyroYawAngle() {
        return m_gyro.getYaw() * DriveSubsystemConstants.GYRO_ORIENTATION;
    }

    @Override
    public double getGyroPitchAngle() {
        return m_gyro.getPitch();
    }

    @Override
    public double getGyroRollAngle() {
        return -m_gyro.getRoll();
    }

    @Override
    public Rotation2d getGyroYawRotation2d() {
        return Rotation2d.fromDegrees(getGyroYawAngle());
    }

    @Override
    public void setGyroAngle(double angle) {
        m_gyro.setAngleAdjustment(angle);
    }

    @Override
    public void reset() {
        m_gyro.reset();
        m_gyro.setAngleAdjustment(0);
    }

    @Override
    public double getVelocityX() {
        return m_gyro.getVelocityX();
    }

    @Override
    public double getVelocityY() {
        return m_gyro.getVelocityY();
    }

    @Override
    public double getRate() {
        return m_gyro.getRate() * DriveSubsystemConstants.GYRO_ORIENTATION;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = m_gyro.isConnected();
        inputs.gyroAngle = getGyroYawAngle();
    }
}
