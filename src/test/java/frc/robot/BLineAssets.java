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
 * Reads the BLine paths under {@code src/main/deploy/autos/paths} so tests can inspect them without
 * constructing a robot.
 *
 * <p>Test support, not a test. {@link RobotContainerSmokeTest} uses {@link
 * #eventKeysReferencedByPaths()} to check that every event trigger drawn in BLine Web has a command
 * registered for its {@code lib_key}.
 *
 * <p>Only {@code path_elements} entries of {@code "type": "event_trigger"} are read, for their
 * {@code lib_key}. Reading nothing else keeps a format change elsewhere in the file from breaking
 * the test.
 */
final class BLineAssets {

  private BLineAssets() {}

  /** Where BLine Web exports, and where BLine-Lib reads from on the robot. */
  private static final Path PATHS_DIR =
      Path.of("src", "main", "deploy", "autos", "paths").toAbsolutePath();

  private static final ObjectMapper MAPPER = new ObjectMapper();

  /**
   * Every {@code lib_key} used by any path. Empty until the team draws one, so a check built on
   * this passes vacuously in the template.
   */
  static Set<String> eventKeysReferencedByPaths() {
    Set<String> names = new LinkedHashSet<>();
    File[] files = PATHS_DIR.toFile().listFiles((dir, name) -> name.endsWith(".json"));
    if (files == null) {
      return names;
    }
    for (File file : Arrays.stream(files).sorted().toList()) {
      for (JsonNode element : parse(file).path("path_elements")) {
        String key = element.path("lib_key").asText("");
        if ("event_trigger".equals(element.path("type").asText()) && !key.isBlank()) {
          names.add(key);
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
              + " as JSON. A path that will not parse is one the robot cannot load —"
              + " re-export it from BLine Web rather than hand-editing it.",
          e);
    }
  }
}
