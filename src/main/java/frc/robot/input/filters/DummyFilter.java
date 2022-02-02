package frc.robot.input.filters;

public class DummyFilter extends DoubleTypeFilter {
    @Override
    public Double applyFilter(Double input) {
        return input;
    }
}