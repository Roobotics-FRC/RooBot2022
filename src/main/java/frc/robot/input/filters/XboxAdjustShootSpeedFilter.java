package frc.robot.input.filters;

/**
 * A Javadoc template. TODO: Update XboxAdjustShootSpeedFilter Javadoc.
 */
public class XboxAdjustShootSpeedFilter extends DoubleTypeFilter {
    @Override
    public Double applyFilter(Double val) {
        return -Math.copySign(Math.pow(val, 2), val);
    }
}
