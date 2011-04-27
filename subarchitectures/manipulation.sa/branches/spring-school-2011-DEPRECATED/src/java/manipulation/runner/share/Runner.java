package manipulation.runner.share;

import manipulation.strategies.Strategy;

/**
 * represents a start up for a strategy
 * 
 * @author Torben Toeniges
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
