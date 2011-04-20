package manipulation.muster.runner.share;

import manipulation.muster.strategies.Strategy;

/**
 * represents a start up for a strategy
 * 
 * @author ttoenige
 * 
 */
public interface Runner {
	/**
	 * starts the given strategy
	 * 
	 * @param strategyName
	 *            name of the strategy to start
	 */
	public void startStrategy(Strategy.Name strategyName);
}
