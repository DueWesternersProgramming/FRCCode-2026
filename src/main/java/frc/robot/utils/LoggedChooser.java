package frc.robot.utils;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class LoggedChooser<T> {
  public record Option<T>(String label, T value) {}

  private final LoggedDashboardChooser<T> chooser;

  public LoggedChooser(String key, Option<T> defaultOption, List<Option<T>> options) {
    SendableChooser<T> sendableChooser = new SendableChooser<>();

    sendableChooser.setDefaultOption(defaultOption.label(), defaultOption.value());

    for (Option<T> option : options) {
      sendableChooser.addOption(option.label(), option.value());
    }

    this.chooser = new LoggedDashboardChooser<>(key, sendableChooser);
  }

  public void addOption(Option<T> option) {
    chooser.addOption(option.label(), option.value());
  }

  public T get() {
    return chooser.get();
  }

  /**
   * Convenience method for questionnaires that store Supplier<T>.
   */
  @SuppressWarnings("unchecked")
  public <R> R getSupplied() {
    return ((Supplier<R>) chooser.get()).get();
  }
}