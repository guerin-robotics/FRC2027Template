// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import frc.lib.FieldConstants;
import frc.robot.PathPlannerAssets.Asset;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Validates the PathPlanner files in {@code src/main/deploy/pathplanner} without building a robot.
 *
 * <p>Autos fail quietly. A path whose file was renamed, a waypoint dragged off the field, a
 * constraint left at zero — none of these throw at deploy time. They surface as an auto that drives
 * somewhere wrong, or does not move, during a match. {@code .claude/rules/03-commands.md} records
 * that the 2026 autos overran their budget and had the last path truncated in *every* match; this
 * test attacks the class of problem one step earlier, at the file level.
 *
 * <p><b>The auto directories are empty in this template</b>, which is correct — 2026's routines
 * were game-specific and were removed. Every test here therefore passes vacuously today. That is a
 * real trap with file-scanning tests, so {@link #reportDiscoveredAssets()} prints the counts on
 * every run: a reviewer sees "0 autos, 0 paths" rather than an unexplained green tick, and the
 * checks begin doing work the moment the first 2027 auto is drawn.
 *
 * <p>The companion check — that every named command an auto references was actually registered —
 * needs the {@code NamedCommands} registry populated, so it lives in {@link
 * RobotContainerSmokeTest} where the container is already built.
 */
class PathPlannerAssetsTest {

  private static List<Asset> autos;
  private static List<Asset> paths;

  @BeforeAll
  static void loadAssets() {
    autos = PathPlannerAssets.autos();
    paths = PathPlannerAssets.paths();
  }

  @Test
  void reportDiscoveredAssets() {
    System.out.println(
        "PathPlanner assets under "
            + PathPlannerAssets.pathPlannerDirectory()
            + ": "
            + autos.size()
            + " auto(s), "
            + paths.size()
            + " path(s)");
    if (autos.isEmpty() && paths.isEmpty()) {
      System.out.println(
          "  No routines yet — every check in this class is passing vacuously. Expected for the"
              + " template; not expected once 2027 autos exist.");
    }
  }

  @Test
  void everyFileParsesAsJson() {
    // PathPlannerAssets throws with the offending filename if a document will not parse. A file
    // that will not parse is one AutoBuilder cannot load, and the robot finds out at deploy.
    assertDoesNotThrow(PathPlannerAssets::autos);
    assertDoesNotThrow(PathPlannerAssets::paths);
  }

  @Test
  void autoAndPathFilesAreNotInSubdirectories() {
    List<String> nested =
        java.util.stream.Stream.concat(autos.stream(), paths.stream())
            .map(Asset::relativePath)
            .filter(relative -> relative.chars().filter(c -> c == '/').count() > 1)
            .toList();

    assertTrue(
        nested.isEmpty(),
        () ->
            "These files are in a subdirectory:\n  "
                + String.join("\n  ", nested)
                + "\n\nPathPlanner's loader lists the autos/ and paths/ directories and skips"
                + " anything that is a directory, so a nested file is invisible to the robot while"
                + " still appearing in the repo and in the GUI. The folders shown in the"
                + " PathPlanner GUI are virtual — recorded in settings.json, not on disk — so a"
                + " real subdirectory here means someone moved files by hand.");
  }

  @Test
  void everyPathReferencedByAnAutoExists() {
    Set<String> available = paths.stream().map(Asset::name).collect(Collectors.toSet());
    List<String> missing =
        PathPlannerAssets.pathsReferencedByAutos().stream()
            .filter(referenced -> !available.contains(referenced))
            .toList();

    assertTrue(
        missing.isEmpty(),
        () ->
            "Autos reference paths that do not exist: "
                + missing
                + "\nAvailable paths: "
                + available.stream().sorted().toList()
                + "\n\nA renamed or deleted .path leaves the reference behind in the .auto."
                + " PathPlannerPath.fromPathFile throws when the auto is built, which takes out"
                + " the whole auto chooser, not just the one routine.");
  }

  @Test
  void everyWaypointIsInsideTheFieldBoundary() {
    double fieldLength = FieldConstants.aprilTagLayout.getFieldLength();
    double fieldWidth = FieldConstants.aprilTagLayout.getFieldWidth();

    List<String> offField = new ArrayList<>();
    for (Asset path : paths) {
      JsonNode waypoints = path.json().path("waypoints");
      for (int i = 0; i < waypoints.size(); i++) {
        JsonNode anchor = waypoints.get(i).path("anchor");
        double x = anchor.path("x").asDouble(Double.NaN);
        double y = anchor.path("y").asDouble(Double.NaN);
        if (Double.isNaN(x) || Double.isNaN(y)) {
          offField.add(path.relativePath() + " waypoint " + i + " has no numeric anchor");
        } else if (x < 0.0 || x > fieldLength || y < 0.0 || y > fieldWidth) {
          offField.add(
              String.format("%s waypoint %d at (%.2f, %.2f)", path.relativePath(), i, x, y));
        }
      }
    }

    assertTrue(
        offField.isEmpty(),
        () ->
            String.format(
                    "These waypoints are outside the %.2f x %.2f m field:%n  ",
                    fieldLength, fieldWidth)
                + String.join("\n  ", offField)
                + "\n\nField dimensions come from FieldConstants.aprilTagLayout, which still points"
                + " at the 2026 field (see CLAUDE.md). If this fails after the 2027 layout lands"
                + " and the paths look right, check that the layout was updated before assuming the"
                + " paths are wrong.");
  }

  @Test
  void everyPathHasUsableConstraints() {
    List<String> unusable = new ArrayList<>();
    for (Asset path : paths) {
      JsonNode constraints = path.json().path("globalConstraints");
      if (constraints.isMissingNode()) {
        unusable.add(path.relativePath() + " has no globalConstraints block");
        continue;
      }
      if (constraints.path("unlimited").asBoolean(false)) {
        // An explicitly unlimited path is a deliberate choice, not a zero left behind.
        continue;
      }
      for (String key :
          List.of(
              "maxVelocity", "maxAcceleration", "maxAngularVelocity", "maxAngularAcceleration")) {
        double value = constraints.path(key).asDouble(Double.NaN);
        if (Double.isNaN(value) || value <= 0.0) {
          unusable.add(path.relativePath() + " has " + key + " = " + constraints.path(key));
        }
      }
    }

    assertTrue(
        unusable.isEmpty(),
        () ->
            "These paths have a constraint that would stall the robot:\n  "
                + String.join("\n  ", unusable)
                + "\n\nA zero max velocity or acceleration is not an error to PathPlanner — the"
                + " profile simply never advances, and the auto sits still until the path-following"
                + " command is interrupted by the end of the auto period.");
  }
}
