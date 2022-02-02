package frc.robot.input.filters;

/**
 * Generic Filter interface
 * All filters shall implement this interface
 * based on a specific type.
 */

public interface GenericFilter<E> {
    public E applyFilter(E input);
}