// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static com.tngtech.archunit.base.DescribedPredicate.describe;
import static com.tngtech.archunit.core.domain.JavaClass.Predicates.assignableTo;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.fields;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.methods;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;

import com.tngtech.archunit.base.DescribedPredicate;
import com.tngtech.archunit.core.domain.JavaClass;
import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.domain.JavaModifier;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;
import com.tngtech.archunit.lang.ArchCondition;
import com.tngtech.archunit.lang.ConditionEvents;
import com.tngtech.archunit.lang.SimpleConditionEvent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Enforces the architecture rules in {@code .claude/rules/} as build failures.
 *
 * <p>Those rules exist because the alternatives were tried and caused problems — they are the
 * accumulated result of two competition seasons. Until now they were enforced only by whoever
 * happened to read them: a reviewer, or an agent that loaded CLAUDE.md. A rule enforced that way
 * degrades quietly, because the pressure to break it always arrives at 11pm the night before an
 * event, and the person breaking it is usually not the person who wrote the rule down.
 *
 * <p>Each test below names the rule file it comes from. If a rule is deliberately changed, change
 * it in {@code .claude/rules/}, in {@code .github/instructions/}, and here — CLAUDE.md already
 * warns that the first two drift apart; this is a third copy and the only one that fails loudly.
 *
 * <h2>Exemptions</h2>
 *
 * <p>{@link #onlyAllianceFlipUtilReadsTheAlliance} carries two named exemptions, both written out
 * in full at the rule with why each one is legitimate rather than deferred. An exemption list is
 * honest where a deleted rule is not: it keeps the rule enforced everywhere else while any debt
 * stays visible and greppable. Add to it only for a call that genuinely is not per-loop, and say so
 * at the line.
 *
 * <h2>What this cannot check</h2>
 *
 * <p>ArchUnit reads bytecode, so it sees types, fields, and call targets. It cannot see whether a
 * value is correct, whether a threshold is sane, or whether a call sits inside a loop. Rules about
 * <i>meaning</i> — encoder offsets, current limits, gains — stay human-reviewed.
 */
class ArchitectureRulesTest {

  private static JavaClasses robotCode;

  @BeforeAll
  static void importRobotCode() {
    // Main sources only. Test classes legitimately break several of these rules — reflecting into
    // private fields, holding subsystem references — and are not what these rules govern.
    robotCode =
        new ClassFileImporter()
            .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
            .importPackages("frc");
  }

  // ============================================================================================
  // 01-architecture.md — AdvantageKit IO layer
  // ============================================================================================

  @Test
  void hardwareIsOnlyTouchedByIoImplementations() {
    // The rule that makes log replay possible. If a subsystem constructs a TalonFX directly, its
    // behavior cannot be reproduced from a match log, because the hardware read is not in the
    // @AutoLog inputs. CLAUDE.md lists "NEVER add hardware calls inside a subsystem class" as a
    // Hard Stop; this is that stop, mechanized.
    noClasses()
        .that()
        .resideInAPackage("frc.robot..")
        .and()
        .resideOutsideOfPackage("frc.robot.generated..")
        .and(describe("are not IO implementations", not(nameContaining("IO"))))
        .should()
        .dependOnClassesThat()
        .resideInAnyPackage("com.ctre.phoenix6.hardware..", "com.revrobotics..")
        .because(
            "hardware belongs behind an XxxIO interface (.claude/rules/01-architecture.md)."
                + " A hardware call in a subsystem cannot be replayed from a log, which is how"
                + " real match bugs were caught in 2026. frc.robot.generated is exempt because"
                + " TunerConstants is Tuner X output and names device types in its own generics.")
        .check(robotCode);
  }

  @Test
  void ioInterfaceMethodsAllHaveDefaultImplementations() {
    // RobotContainer's REPLAY branch builds every IO as `new ModuleIO() {}` / `new GyroIO() {}` /
    // `new VisionIO() {}`. That only compiles while every method on the interface has a body. One
    // abstract method added to an IO interface breaks replay for the whole subsystem, and the
    // error surfaces in RobotContainer rather than at the interface where the mistake was made.
    methods()
        .that()
        .areDeclaredInClassesThat()
        .areInterfaces()
        .and()
        .areDeclaredInClassesThat(describe("are IO interfaces", nameContaining("IO")))
        .and()
        .areDeclaredInClassesThat()
        .resideInAPackage("frc.robot.subsystems..")
        .should()
        .notHaveModifier(JavaModifier.ABSTRACT)
        .because(
            "the replay branch of RobotContainer instantiates every IO interface anonymously"
                + " (`new ModuleIO() {}`), which requires every method to be default")
        .check(robotCode);
  }

  @Test
  void everySubsystemProcessesItsInputs() {
    // Removing Logger.processInputs() is a Hard Stop in CLAUDE.md: without it the subsystem's
    // inputs never reach the log, so the subsystem cannot be replayed and its @AutoLog fields are
    // dead weight. Nothing about the robot misbehaves — the loss is only discovered later, when
    // someone opens a match log to diagnose something else and the channels are not there.
    classes()
        .that()
        .areAssignableTo(SubsystemBase.class)
        .and()
        .resideInAPackage("frc.robot.subsystems..")
        .should(callProcessInputs())
        .because("every periodic() must publish its IO inputs (.claude/rules/00-safety.md)")
        .check(robotCode);
  }

  // ============================================================================================
  // 01-architecture.md — RobotState singleton
  // ============================================================================================

  @Test
  void noSubsystemHoldsAnotherSubsystem() {
    // Shared state goes through RobotState.getInstance() or a supplier passed at construction.
    // Direct references create initialization-order dependencies and circular logic, and they are
    // what makes a subsystem impossible to unit test — VisionFilterTest works precisely because
    // Vision takes a consumer rather than a Drive.
    fields()
        .that()
        .areDeclaredInClassesThat()
        .areAssignableTo(Subsystem.class)
        .and()
        .areDeclaredInClassesThat()
        .resideInAPackage("frc.robot..")
        .should()
        .notHaveRawType(assignableTo(Subsystem.class))
        .because(
            "no subsystem may reference another (.claude/rules/01-architecture.md). Use"
                + " RobotState, or take a supplier in the constructor. RobotContainer is the"
                + " wiring layer and is exempt by not being a Subsystem itself.")
        .check(robotCode);
  }

  // ============================================================================================
  // 01-architecture.md — AllianceFlipUtil
  // ============================================================================================

  @Test
  void onlyAllianceFlipUtilReadsTheAlliance() {
    // DriverStation.getAlliance() allocates an Optional on every call. At 50 Hz that is GC
    // pressure and unpredictable periodic() timing — one of the two failure modes CLAUDE.md
    // carries forward from 2026, where the robot ran nearer 30 Hz than 50 Hz all season.
    // AllianceFlipUtil.shouldFlip() caches the answer once per loop.
    noClasses()
        .that()
        .doNotHaveFullyQualifiedName("frc.lib.AllianceFlipUtil")

        // Exempt, legitimately: runs once when match metadata is first published, not per loop.
        .and()
        .doNotHaveFullyQualifiedName("frc.lib.MatchMetadataLogger")

        // Exempt, legitimately: this is the shouldFlipPath supplier handed to
        // AutoBuilder.configure(). PathPlanner evaluates it in FollowPathCommand.initialize() and
        // PathfindingCommand.initialize() — once when a path starts, to decide whether to mirror
        // it — never in execute(). Verified against PathplannerLib 2026.1.2. Recheck on a major
        // PathPlanner upgrade; if it ever moves into execute() this becomes the same debt as
        // DriveCommands below.
        .and()
        .doNotHaveFullyQualifiedName("frc.robot.subsystems.drive.Drive")
        .should()
        .callMethod(DriverStation.class, "getAlliance")
        .because(
            "getAlliance() allocates per call; use AllianceFlipUtil.shouldFlip(), which caches"
                + " once per loop (.claude/rules/01-architecture.md)")
        .check(robotCode);
  }

  // ============================================================================================
  // 01-architecture.md — Triggers
  // ============================================================================================

  @Test
  void onlyTriggersTouchesAControllerObject() {
    // Controllers are private inside Triggers, and everything else reads named accessors. This is
    // what makes the dashboard controller swap a one-file change, and it prevents the 2026 bug
    // where one command read a stick directly and followed an input nobody was holding while
    // ordinary driving looked fine (docs/drive-controller-mode.md).
    noClasses()
        .that()
        .doNotHaveFullyQualifiedName("frc.robot.Triggers")
        .should()
        .dependOnClassesThat()
        .haveNameMatching(
            "edu\\.wpi\\.first\\.wpilibj2?\\.(command\\.button\\.)?"
                + "(Command)?(XboxController|Joystick|GenericHID|PS4Controller|PS5Controller"
                + "|StadiaController)")
        .because(
            "controller objects are private to Triggers; axes go through driveXSupplier() and"
                + " buttons through named accessors (.claude/rules/01-architecture.md)")
        .check(robotCode);
  }

  // ============================================================================================
  // 03-commands.md — Static command factories
  // ============================================================================================

  @Test
  void commandsAreStaticFactoriesNotClasses() {
    // Static factories compose, are testable, and show their structure in the AdvantageKit
    // command log. A named Command subclass hides composition inside a class body and adds a file
    // per behavior. ContinuousConditionalCommand in frc.lib is a genuine framework primitive —
    // it fills a gap in WPILib's ConditionalCommand — and is not game logic, so it lives outside
    // this package and outside this rule.
    noClasses()
        .that()
        .resideInAPackage("frc.robot.commands..")
        .should()
        .beAssignableTo(Command.class)
        .because(
            "commands are static factory methods returning composed Commands"
                + " (.claude/rules/03-commands.md)")
        .check(robotCode);
  }

  // ============================================================================================
  // Layering
  // ============================================================================================

  @Test
  void theLibraryLayerDoesNotDependOnRobotCode() {
    // frc.lib is the reusable layer that survives a season rollover — this template exists
    // because that separation held. A frc.lib class reaching into a subsystem or RobotState is
    // what turns "copy frc.lib into next year's repo" into an afternoon of untangling.
    noClasses()
        .that()
        .resideInAPackage("frc.lib..")
        .should()
        .dependOnClassesThat(
            describe(
                "are robot code other than Constants",
                javaClass ->
                    javaClass.getPackageName().startsWith("frc.robot")
                        && !javaClass.getName().startsWith("frc.robot.Constants")))
        .because(
            "frc.lib must stay season-agnostic so it carries into the next template."
                + " frc.robot.Constants is the one allowed exception — the tunables and"
                + " PhoenixSignalLogger read currentMode and tuningMode from it.")
        .check(robotCode);
  }

  // ============================================================================================
  // 02-hardware.md / 05-git.md — Diagnostics reach the log, not the console
  // ============================================================================================

  @Test
  void subsystemsDoNotWriteToTheConsole() {
    // Scoped to subsystems on purpose. A println inside a 50 Hz periodic() is both a loop-timing
    // cost and invisible after the match — console output is not in the AdvantageKit log, so it
    // cannot be replayed or scrubbed. Use Logger.recordOutput().
    //
    // Deliberately NOT applied to frc.robot.commands or Robot: the feedforward and wheel-radius
    // characterization routines print their results for someone reading the console during a
    // tuning session, and Robot prints the auto duration once per match. Both are intentional.
    noClasses()
        .that()
        .resideInAPackage("frc.robot.subsystems..")
        .should()
        .accessField(System.class, "out")
        .orShould()
        .accessField(System.class, "err")
        .because(
            "console output is not captured in the match log and costs loop time at 50 Hz;"
                + " use Logger.recordOutput() (.claude/rules/05-git.md)")
        .check(robotCode);
  }

  // ============================================================================================
  // Helpers
  // ============================================================================================

  /** Matches a class whose fully qualified name contains the fragment, inner classes included. */
  private static DescribedPredicate<JavaClass> nameContaining(String fragment) {
    return DescribedPredicate.describe(
        "name containing \"" + fragment + "\"",
        javaClass -> javaClass.getName().contains(fragment));
  }

  private static DescribedPredicate<JavaClass> not(DescribedPredicate<JavaClass> predicate) {
    return DescribedPredicate.not(predicate);
  }

  /** Asserts the class calls {@code Logger.processInputs(...)} somewhere in its own methods. */
  private static ArchCondition<JavaClass> callProcessInputs() {
    return new ArchCondition<>("call Logger.processInputs(...)") {
      @Override
      public void check(JavaClass subsystem, ConditionEvents events) {
        boolean processesInputs =
            subsystem.getMethodCallsFromSelf().stream()
                .anyMatch(
                    call ->
                        call.getTargetOwner()
                                .getFullName()
                                .equals("org.littletonrobotics.junction.Logger")
                            && call.getName().equals("processInputs"));
        if (!processesInputs) {
          events.add(
              SimpleConditionEvent.violated(
                  subsystem,
                  subsystem.getName()
                      + " never calls Logger.processInputs(...). Its IO inputs will not appear in"
                      + " the log and the subsystem cannot be replayed. Nothing about the robot"
                      + " will misbehave — this is only discovered when someone opens a match log"
                      + " looking for the channels and they are absent."));
        }
      }
    };
  }
}
