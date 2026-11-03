package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * Formato das abas 2D e 3D de todos os layouts, contra as listas do
 * AdvantageScope 26 (Field2dController_Config e Field3dController_Config).
 *
 * O AdvantageScope abre um layout errado sem reclamar: a fonte só não aparece.
 * O caso mais traiçoeiro é o filho (linhas até as tags, vetores dos módulos,
 * peças do robô): ele pertence à fonte logo acima, então depois de um ghost sai
 * do fantasma e depois de uma trajetória não é desenhado.
 */
class AdvantageScopeLayoutFormatTest {

    private static final Path LAYOUTS = Path.of("config/advantagescope/layouts");
    private static final Path ROBOT_MODEL = Path.of("config/advantagescope/mecanismos/Robot_Robo_9460/config.json");

    private static final int TAB_2D = 2;
    private static final int TAB_3D = 3;

    private static final Set<String> POSES = Set.of("Pose2d", "Pose3d", "Pose2d[]", "Pose3d[]",
        "Transform2d", "Transform3d", "Transform2d[]", "Transform3d[]");
    private static final Set<String> POINTS = plus(POSES, "Translation2d", "Translation3d",
        "Translation2d[]", "Translation3d[]");
    private static final Set<String> PATHS = Set.of("Pose2d[]", "Pose3d[]", "Transform2d[]", "Transform3d[]",
        "Translation2d[]", "Translation3d[]", "SwerveSample[]", "DifferentialSample[]", "Trajectory");
    private static final Set<String> MODULES = Set.of("SwerveModuleState[]");
    private static final Set<String> ROTATIONS = Set.of("Rotation2d", "Rotation3d");
    private static final Set<String> PIECES = Set.of("Pose3d", "Pose3d[]", "Transform3d", "Transform3d[]",
        "Translation3d", "Translation3d[]");

    private record Rule(Set<String> logTypes, Set<String> options, boolean child) {
        static Rule root(Set<String> logTypes, String... options) {
            return new Rule(logTypes, Set.of(options), false);
        }

        static Rule child(Set<String> logTypes, String... options) {
            return new Rule(logTypes, Set.of(options), true);
        }
    }

    private static final Map<String, Rule> RULES_2D = Map.of(
        "robot", Rule.root(POSES, "bumpers"),
        "ghost", Rule.root(POSES, "color"),
        "vision", Rule.child(POINTS, "color", "size"),
        "swerveStates", Rule.child(MODULES, "color", "arrangement"),
        "rotationOverride", Rule.child(ROTATIONS),
        "trajectory", Rule.root(PATHS, "color", "size"),
        "heatmap", Rule.root(POINTS, "timeRange"),
        "arrow", Rule.root(POSES, "position"));

    private static final Map<String, Rule> RULES_3D = Map.ofEntries(
        Map.entry("robot", Rule.root(POSES, "model")),
        Map.entry("ghost", Rule.root(POSES, "model", "color")),
        Map.entry("component", Rule.child(Set.of("Pose3d", "Pose3d[]"))),
        Map.entry("mechanism", Rule.child(Set.of("Mechanism2d"), "plane")),
        Map.entry("vision", Rule.child(POINTS, "color", "size")),
        Map.entry("swerveStates", Rule.child(MODULES, "color", "arrangement")),
        Map.entry("rotationOverride", Rule.child(ROTATIONS)),
        Map.entry("gamePiece", Rule.root(PIECES, "variant")),
        Map.entry("trajectory", Rule.root(PATHS, "color", "size")),
        Map.entry("heatmap", Rule.root(POINTS, "timeRange")),
        Map.entry("aprilTag", Rule.root(Set.of("Pose3d", "Pose3d[]"), "variant")),
        Map.entry("aprilTagIDs", Rule.child(Set.of("NumberArray"))),
        Map.entry("axes", Rule.root(Set.of("Pose3d", "Pose3d[]"))),
        Map.entry("cone", Rule.root(plus(POSES, "Trajectory"), "color", "position")),
        Map.entry("cameraOverride", Rule.root(Set.of("Pose3d", "Transform3d"))));

    private static Set<String> plus(Set<String> base, String... extra) {
        Set<String> all = new HashSet<>(base);
        all.addAll(List.of(extra));
        return Set.copyOf(all);
    }

    private static List<Path> layoutFiles() throws Exception {
        try (var files = Files.list(LAYOUTS)) {
            return files.filter(file -> file.toString().endsWith(".json")).sorted().toList();
        }
    }

    private static List<JsonNode> sources(JsonNode tab) {
        List<JsonNode> list = new ArrayList<>();
        tab.get("controller").get("sources").forEach(list::add);
        return list;
    }

    private static List<JsonNode> tabs(Path layout) throws Exception {
        List<JsonNode> list = new ArrayList<>();
        for (JsonNode hub : new ObjectMapper().readTree(layout.toFile()).get("hubs")) {
            hub.get("state").get("tabs").get("tabs").forEach(list::add);
        }
        return list;
    }

    /** O pai de um filho é a primeira fonte acima dele que não é filho. */
    private static boolean hasParent(List<JsonNode> list, int index, Map<String, Rule> rules) {
        String childType = list.get(index).get("type").asText();
        for (int i = index - 1; i >= 0; i--) {
            String type = list.get(i).get("type").asText();
            Rule rule = rules.get(type);
            if (rule != null && rule.child()) continue;
            return "aprilTagIDs".equals(childType)
                ? "aprilTag".equals(type)
                : "robot".equals(type) || "ghost".equals(type);
        }
        return false;
    }

    private static void check(String where, List<JsonNode> list, int index, Map<String, Rule> rules, Set<String> problems) {
        JsonNode source = list.get(index);
        String type = source.get("type").asText();
        String description = where + " -> " + type + " " + source.get("logKey").asText();
        Rule rule = rules.get(type);
        if (rule == null) {
            problems.add(description + ": tipo não existe nesta aba");
            return;
        }
        if (!rule.logTypes().contains(source.get("logType").asText())) {
            problems.add(description + ": não aceita " + source.get("logType").asText());
        }
        for (String option : rule.options()) {
            if (!source.path("options").has(option)) {
                problems.add(description + ": falta a opção " + option);
            }
        }
        if (rule.child() && !hasParent(list, index, rules)) {
            problems.add(description + ": filho sem robot ou ghost logo acima");
        }
    }

    @Test
    void everyFieldSourceIsOneAdvantageScopeDraws() throws Exception {
        Set<String> problems = new TreeSet<>();
        int checked = 0;
        for (Path layout : layoutFiles()) {
            for (JsonNode tab : tabs(layout)) {
                int type = tab.get("type").asInt();
                if (type != TAB_2D && type != TAB_3D) continue;
                List<JsonNode> list = sources(tab);
                String where = layout.getFileName() + " / " + tab.get("title").asText();
                for (int i = 0; i < list.size(); i++) {
                    check(where, list, i, type == TAB_2D ? RULES_2D : RULES_3D, problems);
                    checked++;
                }
            }
        }
        assertTrue(checked > 100, "Poucas fontes conferidas: " + checked);
        assertTrue(problems.isEmpty(), "Fontes que não aparecem no AdvantageScope:\n" + String.join("\n", problems));
    }

    @Test
    void tagLinesLeaveFromTheEstimatedRobot() throws Exception {
        // Depois de um ghost, as linhas sairiam do fantasma e não da pose estimada.
        Set<String> wrong = new TreeSet<>();
        for (JsonNode tab : tabs(LAYOUTS.resolve("FRC9460-Campo2D3D.json"))) {
            int type = tab.get("type").asInt();
            if (type != TAB_2D && type != TAB_3D) continue;
            List<JsonNode> list = sources(tab);
            Map<String, Rule> rules = type == TAB_2D ? RULES_2D : RULES_3D;
            for (int i = 0; i < list.size(); i++) {
                if (!rules.get(list.get(i).get("type").asText()).child()) continue;
                int parent = i;
                while (rules.get(list.get(parent).get("type").asText()).child()) parent--;
                if (!"robot".equals(list.get(parent).get("type").asText())) {
                    wrong.add(tab.get("title").asText() + " -> " + list.get(i).get("logKey").asText());
                }
            }
        }
        assertTrue(wrong.isEmpty(), "Filhos fora do robô: " + wrong);
    }

    @Test
    void limelightViewPointsToACameraOfTheRobotModel() throws Exception {
        int cameras = new ObjectMapper().readTree(ROBOT_MODEL.toFile()).get("cameras").size();
        for (JsonNode tab : tabs(LAYOUTS.resolve("FRC9460-Campo2D3D.json"))) {
            if (tab.get("type").asInt() != TAB_3D) continue;
            int camera = tab.get("renderer").get("cameraIndex").asInt();
            assertTrue(camera < cameras, tab.get("title").asText() + " usa a câmera " + camera);
        }
    }
}
