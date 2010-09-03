/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package celm.monitor;

import java.util.Map;
import java.util.StringTokenizer;

import Ice.ObjectImpl;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMEntrySet;

/**
 *
 * @author harmish khambhaita
 */
public class CelmCastComponent extends ManagedComponent
{
    CelmWriter celmWriter;
    WMEntrySet entrySet;

    public CelmCastComponent()
    {
        super();
        entrySet = WMEntrySet.create(this);
        celmWriter = new CelmWriter();

        entrySet.setHandler(celmWriter);
    }

    @Override
    protected void start()
    {
        entrySet.start();
        super.start();
    }

    @Override
    protected void configure(Map<String, String> _config)
    {
        super.configure(_config);
        String subscrStr = _config.get("--subscribe");
        if (subscrStr != null)
        {
            StringTokenizer st = new StringTokenizer(subscrStr, ",");
            while (st.hasMoreTokens())
            {
                String className = st.nextToken();

                try
                {
                    System.out.println("add type '" + className + "'");
                    ClassLoader.getSystemClassLoader().loadClass(className);
                    entrySet.addType((Class<? extends ObjectImpl>) Class.forName(className));
                }
                catch(ClassNotFoundException e)
                {
                    println("trying to register for class that doesn't exist!");
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    protected void stop()
    {
        super.stop();
    }
}
