package frc.robot.path;

import jaci.pathfinder.Trajectory;
import lombok.RequiredArgsConstructor;

import javax.annotation.Nonnull;

@RequiredArgsConstructor
public class PlannedPath {
    private final Trajectory trajectory;
    private int index;

    @Nonnull
    public Trajectory.Segment getSegmentAndIncr() {
        Trajectory.Segment[] segments = this.trajectory.segments;
        return segments[this.index];
    }

    public boolean isComplete() {
        Trajectory.Segment[] segments = this.trajectory.segments;
        return this.index >= segments.length;
    }

    public void reset() {
        this.index = 0;
    }
}
