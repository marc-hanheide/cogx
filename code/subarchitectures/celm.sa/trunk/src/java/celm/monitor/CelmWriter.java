/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package celm.monitor;

import Ice.ConnectionI.StartCallback;
import java.util.Map;

import Ice.ObjectImpl;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEntrySet.ChangeHandler;
import castutils.viewer.plugins.Plugin;
import celmarchitecture.subarchitectures.elmwriter.ElmWriter;

/**
 *
 * @author harmish
 */
public class CelmWriter implements ChangeHandler
{
    ElmWriter elmWriter = new ElmWriter();

    public CelmWriter()
    {
        elmWriter.start();
    }

    @Override
    public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map, WorkingMemoryChange wmc,
            ObjectImpl newEntry, ObjectImpl oldEntry) throws CASTException
    {
        switch (wmc.operation)
        {
            case ADD:
                System.out.println("got something to add on elm");
                break;
            case DELETE:
                System.out.println("got something to delete on elm");
                break;
            case GET:
                System.out.println("got something to get on elm");
                break;
            case OVERWRITE:
                System.out.println("got something to overwrite on elm");
                break;
            case WILDCARD:
                System.out.println("got something to wildcard on elm");
                break;
        }
        //throw new UnsupportedOperationException("Not supported yet.");
    }

}
