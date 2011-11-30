package spatial.demo;

import cast.architecture.ManagedComponent;
import castutils.castextensions.CASTHelper;

public class ShakeHeadBehaviour extends CASTHelper implements Runnable {

	private Object m_shakeSignal = new Object();
	private boolean m_shaking;
	private boolean m_kill;
	final private PTZMover ptzMover;

	public ShakeHeadBehaviour(ManagedComponent _c) {
		super(_c);
		m_shaking = false;
		m_kill = false;
		ptzMover = new PTZMover(_c);
	}

	public void startShaking() {
		m_shaking = true;
		synchronized (m_shakeSignal) {
			m_shakeSignal.notify();
		}
		log("starting patrol");
	}

	public void stopPatrolling() {
		m_shaking = false;
	}

	public void kill() {
		m_kill = true;
		synchronized (m_shakeSignal) {
			m_shakeSignal.notify();
		}

	}

	@Override
	public void run() {

		while (!m_kill) {

			log("loop");

			// if patrolling is active
			if (m_shaking) {

				log("in shake head");

				ptzMover.moveToRandomPose();
				try {
					Thread.sleep((long) (3000+Math.random()*10000));
				} catch (InterruptedException e) {
					component.logException(e);
				}
			} else {
				log("not in shake head");
				synchronized (m_shakeSignal) {
					try {
						m_shakeSignal.wait();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

		}

	}
}
