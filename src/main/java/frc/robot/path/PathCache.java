package frc.robot.path;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.HashMap;
import java.util.Map;

public class PathCache {
    private final Map<String, Path> cache =
            new HashMap<>();

    public void init() {
        // TODO: Init cache
    }

    public void populate(SendableChooser<Path> chooser) {
        cache.forEach(chooser::addOption);

        this.cache.entrySet()
                .stream()
                .findFirst()
                .ifPresent(entry -> chooser.setDefaultOption(entry.getKey(), entry.getValue()));
    }
}
