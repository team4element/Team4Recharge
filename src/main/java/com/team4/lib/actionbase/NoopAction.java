package com.team4.lib.actionbase;

public class NoopAction implements Action {

    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void stop() {}
}