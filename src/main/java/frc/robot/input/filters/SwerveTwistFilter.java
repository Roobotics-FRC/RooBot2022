package frc.robot.input.filters;

public class SwerveTwistFilter extends DoubleTypeFilter {
    @Override
    public Double applyFilter(Double val) {
        return Math.pow(val, 5);
    }
}
