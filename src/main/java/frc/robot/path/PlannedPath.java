package frc.robot.path;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import javax.annotation.Nonnull;

@RequiredArgsConstructor
public class PlannedPath {
    @Getter
    private final String name;
    private final Trajectory trajectory;
    private int index;

    @Nonnull
    public Segment getSegmentAndIncr() {
        Segment[] segments = this.trajectory.segments;
        return segments[this.index];
    }

    public boolean isComplete() {
        Segment[] segments = this.trajectory.segments;
        return this.index >= segments.length;
    }

    public void reset() {
        this.index = 0;
    }
}
