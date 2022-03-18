package frc.robot.input.filters;

public class XboxWinchFilter extends DoubleTypeFilter {
    @Override
    public Double applyFilter(Double val) {
        return val/2;
    }
}
