package com.team4.lib.actionbase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Composite action, running all sub-actions at the same time All actions are started then updated until all actions
 * report being done.
 */
public class ParallelAction implements Action {
    private final ArrayList<Action> mActions;

    public ParallelAction(List<Action> actions) {
        mActions = new ArrayList<>(actions);
    }

    public ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public void start() {
        mActions.forEach(a -> a.start());
    }

    @Override
    public void update() {
        mActions.forEach(a -> a.update());
    }

    @Override
    public boolean isFinished() {
        for (Action action : mActions) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void stop() {
        mActions.forEach(a -> a.stop());
    }
}