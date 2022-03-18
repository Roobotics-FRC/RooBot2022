package frc.robot.input.filters;

public class LogitechSliderAxisFilter extends DoubleTypeFilter {
    @Override
    public Double applyFilter(Double val) {
        return (1 - val) / 2d;
    }
}
