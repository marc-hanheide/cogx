package cast.examples;

import cast.CASTException;
import cast.core.CASTComponent;
import cast.examples.autogen.WordServer;
import cast.examples.autogen.WordServerPrx;

public class WordServerClient extends CASTComponent {

	@Override
	protected void runComponent() {
		try {
			WordServerPrx agg = getIceServer("aggregate.server",
					WordServer.class, WordServerPrx.class);
			println(agg.getNewWord());
			WordServerPrx impl = getIceServer("implements.server",
					WordServer.class, WordServerPrx.class);
			assert(impl != null);
			
			println(impl.getNewWord());

		} catch (CASTException e) {
			e.printStackTrace();
		}
	}

}
