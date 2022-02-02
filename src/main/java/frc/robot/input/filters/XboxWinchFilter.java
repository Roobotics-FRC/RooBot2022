package frc.robot.input.filters;

/**
 * A Javadoc template. TODO: Update XboxWinchFilter Javadoc.
 */
public class XboxWinchFilter extends DoubleTypeFilter {
    @Override
    public Double applyFilter(Double val) {
        return val/2;
    }
}
