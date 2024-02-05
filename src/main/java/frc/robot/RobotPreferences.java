// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import java.util.Set;

/** Utility class for managing preferences. */
public class RobotPreferences {

  private RobotPreferences() {
    throw new IllegalStateException("RobotPreferences Utility class");
  }

  /** Key - default value pair to support initialization and reset of groups of preferences. */
  public static class PreferenceKeyValue {
    String keyString;
    double defaultValue;

    public PreferenceKeyValue(String keyString, double defaultValue) {
      this.keyString = keyString;
      this.defaultValue = defaultValue;
    }

    public String getKey() {
      return this.keyString;
    }

    public double getDefault() {
      return this.defaultValue;
    }

    public double getValue() {
      return Preferences.getDouble(this.keyString, this.defaultValue);
    }
  }

  /** Reset the Preferences table to default values. */
  public static void resetPreferences() {

    // Reset the arm subsystem preferences
    resetPreferencesArray(Constants.MotorConstants.getMotorPreferences());
  }

  /** Reset an array of Preferences to default values. */
  public static void resetPreferencesArray(PreferenceKeyValue[] prefPairs) {
    for (PreferenceKeyValue keyValue : prefPairs) {
      Preferences.setDouble(keyValue.getKey(), keyValue.getDefault());
    }
  }

  /**
   * Put tunable values into the Preferences table using default values, if the keys don't already
   * exist.
   */
  public static void initPreferencesArray(PreferenceKeyValue[] prefPairs) {
    for (PreferenceKeyValue keyValue : prefPairs) {
      Preferences.initDouble(keyValue.getKey(), keyValue.getDefault());
    }
  }

  /** Log the values of all entries in the Preferences table. */
  public static void logPreferences() {
    NetworkTable prefTable = NetworkTableInstance.getDefault().getTable("Preferences");
    Set<String> prefKeys = prefTable.getKeys();

    for (String keyName : prefKeys) {
      NetworkTableType prefType = prefTable.getEntry(keyName).getType();

      if (prefType == NetworkTableType.kDouble) {
        DataLogManager.log(
            "Preferences/" + keyName + ": " + prefTable.getEntry(keyName).getDouble(-1));
      } else if (prefType == NetworkTableType.kBoolean) {
        DataLogManager.log(
            "Preferences/" + keyName + ": " + prefTable.getEntry(keyName).getBoolean(false));
      }
    }
  }
}
