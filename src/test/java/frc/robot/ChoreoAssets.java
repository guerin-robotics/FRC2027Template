package frc.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * Reads the Choreo trajectories under {@code src/main/deploy/choreo} so tests can inspect them
 * without constructing a robot.
 *
 * <p>Test support, not a test. {@link RobotContainerSmokeTest} uses {@link
 * #eventMarkersReferencedByTrajectories()} to check that every event marker drawn in the Choreo GUI
 * has a command bound to it.
 *
 * <p>Only the top-level {@code "events": [{"name": ...}]} array of each {@code .traj} is read. That
 * is where Choreo stores markers, and reading nothing else keeps a format change elsewhere in the
 * file from breaking the test.
 */
final class ChoreoAssets {

  private ChoreoAssets() {}

  /** Where the Choreo GUI writes, and where ChoreoLib reads from on the robot. */
  private static final Path CHOREO_DIR =
      Path.of("src", "main", "deploy", "choreo").toAbsolutePath();

  private static final ObjectMapper MAPPER = new ObjectMapper();

  /**
   * Every event-marker name used by any trajectory. Empty until the team draws one, so a check
   * built on this passes vacuously in the template.
   */
  static Set<String> eventMarkersReferencedByTrajectories() {
    Set<String> names = new LinkedHashSet<>();
    File[] files = CHOREO_DIR.toFile().listFiles((dir, name) -> name.endsWith(".traj"));
    if (files == null) {
      return names;
    }
    for (File file : Arrays.stream(files).sorted().toList()) {
      JsonNode events = parse(file).path("events");
      for (JsonNode event : events) {
        String name = event.path("name").asText("");
        if (!name.isBlank()) {
          names.add(name);
        }
      }
    }
    return names;
  }

  private static JsonNode parse(File file) {
    try {
      return MAPPER.readTree(file);
    } catch (IOException e) {
      throw new UncheckedIOException(
          "Could not parse "
              + file
              + " as JSON. A trajectory that will not parse is one the robot cannot load —"
              + " re-export it from the Choreo GUI rather than hand-editing it.",
          e);
    }
  }
}
