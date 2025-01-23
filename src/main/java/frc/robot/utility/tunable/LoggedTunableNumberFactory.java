package frc.robot.utility.tunable;

public class LoggedTunableNumberFactory {
  private final String key;

  private boolean defaultTuningEnabled;

  /**
   * Create a new LoggedTunableNumberGroup
   *
   * @param key Key on dashboard
   */
  public LoggedTunableNumberFactory(String key) {
    this(key, true);
  }

  /**
   * Create a new LoggedTunableNumberGroup
   *
   * @param key Key on dashboard
   * @param defaultTuningEnabled Default tuning enabled status
   */
  public LoggedTunableNumberFactory(String key, boolean defaultTuningEnabled) {
    this.key = key;
    this.defaultTuningEnabled = defaultTuningEnabled;
  }

  // --- Create Number ---

  /**
   * Add a number to the group
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber getNumber(String dashboardKey, double defaultValue) {
    return getNumber(dashboardKey, defaultValue, defaultTuningEnabled);
  }

  /**
   * Add a number to the group
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   * @param tuningEnabled Whether the number is enabled
   */
  public LoggedTunableNumber getNumber(
      String dashboardKey, double defaultValue, boolean tuningEnabled) {
    return new LoggedTunableNumber(key + "/" + dashboardKey, defaultValue, tuningEnabled);
  }

  // --- Create Subgroup ---

  /**
   * Add a subgroup to the group
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumberFactory getSubgroup(String dashboardKey) {
    return new LoggedTunableNumberFactory(key + "/" + dashboardKey, defaultTuningEnabled);
  }

  /**
   * Add a subgroup to the group
   *
   * @param dashboardKey Key on dashboard
   * @param defaultTuningEnabled Default tuning enabled status
   */
  public LoggedTunableNumberFactory getSubgroup(String dashboardKey, boolean defaultTuningEnabled) {
    return new LoggedTunableNumberFactory(key + "/" + dashboardKey, defaultTuningEnabled);
  }
}
