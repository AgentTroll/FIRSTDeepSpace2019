package frc.robot.path;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import lombok.RequiredArgsConstructor;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

@RequiredArgsConstructor
public class PathCache {
    private static final Path PATHS_DIR = Paths.get("./paths");

    private final Robot robot;
    private final Map<String, PlannedPath> cache =
            new HashMap<>();

    public void init() {
        try {
            Files.walk(PATHS_DIR).forEach(this::parsePath);
        } catch (IOException e) {
            this.robot.getLogger().error("Failed to initialize PathCache");
            e.printStackTrace();
        }
    }

    private void parsePath(Path path) {
        String fileName = path.getFileName().toString();
        if (!fileName.endsWith(".csv")) {
            this.robot.getLogger().log("Found non-CSV in paths dir: " + fileName);
            return;
        }

        Trajectory traj = Pathfinder.readFromCSV(path.toFile());
        PlannedPath p = new PlannedPath(traj);

        this.cache.put(fileName.substring(0, fileName.indexOf('.')), p);
        this.robot.getLogger().log("Parsed path file: " + fileName);
    }

    public void populate(SendableChooser<PlannedPath> chooser) {
        if (this.cache.isEmpty()) {
            this.robot.getLogger().error("No available paths, auton will fail");
            return;
        }

        this.cache.forEach(chooser::addOption);
        this.cache.entrySet()
                .stream()
                .findFirst()
                .ifPresent(entry -> chooser.setDefaultOption(entry.getKey(), entry.getValue()));
    }
}
