package frc.robot.autonomous;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Class designed to help make dynamic autonomous clear to the technician when they're setting up
 * before the match.
 *
 * <p>You have to provide a default option, ideally `Boolean.FALSE` if it's a if/else. Then provide
 * a List<T> of what options you should include as additional options.
 */
public class AutonomousQuestionaire<T> {
  public record Option<T>(String label, T value) {}

  private final LoggedDashboardChooser<T> chooser;

  public AutonomousQuestionaire(String key, Option<T> defaultOption, List<Option<T>> options) {
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