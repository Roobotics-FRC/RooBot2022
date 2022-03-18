package frc.robot.input.filters;


public class XboxAdjustShootSpeedFilter extends DoubleTypeFilter {
    @Override
    public Double applyFilter(Double val) {
        return -Math.copySign(Math.pow(val, 2), val);
    }
}
