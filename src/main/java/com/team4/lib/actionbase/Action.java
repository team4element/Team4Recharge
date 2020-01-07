package com.team4.lib.actionbase;

/**
 * Base Action interface, most commonly used in AutoModeBase when handeling the auto mode action
 */
public interface Action {
    /**
     * Run code once when the action is started, for setup
     */
    void start();

    /**
     * Called by runAction in AutoModeBase iteratively until the auto mode is finished. Iterative logic lives in this method
     */
    void update();

    /**
     * Returns whether or not the code has finished execution. When implementing this interface, this method is used by
     * the runAction method every cycle to know when to stop running the action
     */
    boolean isFinished();

    /**
     * Run code once when the action finishes, usually for clean up
     */
    void stop();
}