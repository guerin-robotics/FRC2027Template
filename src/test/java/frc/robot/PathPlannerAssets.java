// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * Reads the PathPlanner files under {@code src/main/deploy/pathplanner} so tests can inspect them
 * without constructing a robot.
 *
 * <p>Test support, not a test — {@link PathPlannerAssetsTest} validates the files, and {@link
 * RobotContainerSmokeTest} uses {@link #namedCommandsReferencedByAutos()} to check that every
 * command an auto asks for was actually registered.
 *
 * <p><b>Why this walks the JSON generically.</b> The {@code .auto} format nests commands
 * arbitrarily deep — a {@code sequential} containing a {@code deadline} containing a {@code named}
 * — and PathPlanner has changed that nesting between seasons. Rather than model the schema, {@link
 * #collectCommandData} recurses over every object in the document and picks out the two shapes that
 * matter: {@code {"type": "named", "data": {"name": ...}}} and {@code {"type": "path", "data":
 * {"pathName": ...}}}. Those two keys have been stable across seasons, and a format change
 * elsewhere in the file cannot break this.
 */
final class PathPlannerAssets {

  private PathPlannerAssets() {}

  /** Where the PathPlanner GUI writes, and where {@code Filesystem.getDeployDirectory()} reads. */
  private static final Path PATHPLANNER_DIR =
      Path.of("src", "main", "deploy", "pathplanner").toAbsolutePath();

  private static final ObjectMapper MAPPER = new ObjectMapper();

  /** An {@code .auto} or {@code .path} file, with its parsed contents. */
  record Asset(File file, JsonNode json) {
    /** Name PathPlanner would refer to this file by — the filename without its extension. */
    String name() {
      String fileName = file.getName();
      return fileName.substring(0, fileName.lastIndexOf('.'));
    }

    /** Path relative to the pathplanner directory, for readable failure messages. */
    String relativePath() {
      return PATHPLANNER_DIR.relativize(file.toPath()).toString().replace('\\', '/');
    }
  }

  static Path pathPlannerDirectory() {
    return PATHPLANNER_DIR;
  }

  static List<Asset> autos() {
    return load("autos", ".auto");
  }

  static List<Asset> paths() {
    return load("paths", ".path");
  }

  /**
   * Every named command any auto references.
   *
   * <p>These are the strings that must have been passed to {@code NamedCommands.registerCommand}
   * before {@code AutoBuilder.buildAutoChooser()} ran. One that was not registered does not fail —
   * PathPlanner substitutes {@code Commands.none()} and the auto drives its paths while the
   * mechanism silently does nothing.
   */
  static Set<String> namedCommandsReferencedByAutos() {
    Set<String> names = new LinkedHashSet<>();
    for (Asset auto : autos()) {
      names.addAll(collectCommandData(auto.json(), "named", "name"));
    }
    return names;
  }

  /** Every path name any auto references, via a {@code path} command. */
  static Set<String> pathsReferencedByAutos() {
    Set<String> names = new LinkedHashSet<>();
    for (Asset auto : autos()) {
      names.addAll(collectCommandData(auto.json(), "path", "pathName"));
    }
    return names;
  }

  /**
   * Recursively finds every {@code {"type": <commandType>, "data": {<dataKey>: <value>}}} in the
   * document and returns the values.
   */
  private static List<String> collectCommandData(
      JsonNode node, String commandType, String dataKey) {
    List<String> found = new ArrayList<>();
    if (node.isObject()) {
      JsonNode type = node.get("type");
      JsonNode data = node.get("data");
      if (type != null
          && commandType.equals(type.asText())
          && data != null
          && data.hasNonNull(dataKey)) {
        found.add(data.get(dataKey).asText());
      }
    }
    node.forEach(child -> found.addAll(collectCommandData(child, commandType, dataKey)));
    return found;
  }

  /**
   * Loads every file with the given extension, recursively.
   *
   * <p>The recursion is deliberate even though PathPlanner itself only reads the top level — see
   * {@code PathPlannerAssetsTest.autoAndPathFilesAreNotInSubdirectories}, which uses the difference
   * to catch a file the robot would silently ignore.
   */
  private static List<Asset> load(String subdirectory, String extension) {
    File root = PATHPLANNER_DIR.resolve(subdirectory).toFile();
    List<Asset> assets = new ArrayList<>();
    collectFiles(root, extension, assets);
    assets.sort(Comparator.comparing(Asset::relativePath));
    return assets;
  }

  private static void collectFiles(File directory, String extension, List<Asset> into) {
    File[] entries = directory.listFiles();
    if (entries == null) {
      return;
    }
    for (File entry : Arrays.stream(entries).sorted().toList()) {
      if (entry.isDirectory()) {
        collectFiles(entry, extension, into);
      } else if (entry.getName().endsWith(extension)) {
        into.add(new Asset(entry, parse(entry)));
      }
    }
  }

  private static JsonNode parse(File file) {
    try {
      return MAPPER.readTree(file);
    } catch (IOException e) {
      throw new UncheckedIOException(
          "Could not parse "
              + file
              + " as JSON. A PathPlanner file that will not parse is one"
              + " the robot cannot load — check whether it was hand-edited or truncated.",
          e);
    }
  }
}
