/*
 * @(#)MetaPragma.java	1.14 03/12/19
 *
 * Copyright 2004 Sun Microsystems, Inc. All rights reserved.
 * SUN PROPRIETARY/CONFIDENTIAL. Use is subject to license terms.
 */
/*
 *  COMPONENT_NAME: shasta
 *
 *  ORIGINS: 27
 *
 * Licensed Materials - Property of IBM
 * 5639-D57 (C) COPYRIGHT International Business Machines Corp. 1997,1998,1999
 * RMI-IIOP v1.0
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with IBM Corp.
 *
 *  @(#)MetaPragma.java	1.14 03/12/19
 *
 */

package org.cognitivesystems.idl.som.idlemit;
import java.util.Vector;

import org.cognitivesystems.idl.som.cff.Messages;
/**
 * This is an implementation that handles 
 * #pragma meta scoped_name string
 * where
 * <UL>
 * <LI>    scoped_name ==  "::" separated scoped name
 * <LI>    string ==  separated identifiers, such as "localonly",
 *          "abstract", or "init". 
 *         D59407: NOTE: any non-white-space is grouped
 *          as part of the identifier.
 * </UL>
 *
 * This pragma handler places a vector of Strings into the dynamicVariable()
 * part of the SymtabEntry. The key to access the dynamicVariable()
 * is org.cognitivesystems.idl.som.idlemit.MetaPragma.metaKey
 *
 * It is possible to associate a meta pragma with a forward entry.
 * At some point after the parser has completed, 
 * the method processForward(ForwardEntry entry) should be called 
 * for each ForwardEntry so that the meta information can be folded from 
 * the ForwardEntry into the corresponding InterfaceEntry.
 */
public class MetaPragma extends org.cognitivesystems.idl.PragmaHandler {
    /* Class variables */

    /* key to access the Cached meta info in com.sun.tools.corba.se.idl.SymtabEntry */
    public static int metaKey = org.cognitivesystems.idl.SymtabEntry.getVariableKey();


    /**
     * Main entry point for the MetaPragma handler
     * @param pragma string for pragma name
     * @param currentToken next token in the input stream.
     * @return true if this is a meta pragma.
     */
    public boolean process(String pragma, String currentToken) {
        if ( !pragma.equals("meta"))
            return false;

        org.cognitivesystems.idl.SymtabEntry entry ;
        String msg;
        try {
            entry = scopedName();
            if ( entry == null){
                /* scoped name not found */
                parseException(Messages.msg("idlemit.MetaPragma.scopedNameNotFound"));
                skipToEOL();
            }
            else {
                msg = (currentToken()+ getStringToEOL());
// System.out.println(entry + ":  " + msg);
                Vector v;
                v = (Vector) entry.dynamicVariable(metaKey);
                if ( v== null){
                    v = new Vector();
                    entry.dynamicVariable(metaKey, v);
                }
                parseMsg(v, msg);
           }
        } catch(Exception e){
// System.out.println("exception in MetaPragma");
        }
        return true;
    }


    /**
     * Fold the meta info from the forward entry into its corresponding
     * interface entry.
     * @param forwardEntry the forward entry to process
     */
    static public void processForward(org.cognitivesystems.idl.ForwardEntry forwardEntry){

        Vector forwardMeta;
        try {
            forwardMeta = (Vector)forwardEntry.dynamicVariable(metaKey);
        } catch (Exception e){
            forwardMeta = null;
        }
        org.cognitivesystems.idl.SymtabEntry forwardInterface = forwardEntry.type();
        if (forwardMeta != null && forwardInterface!= null) {
            Vector interfaceMeta;
            try {
                 interfaceMeta= (Vector)forwardInterface.dynamicVariable(metaKey);
            } catch ( Exception e){
                 interfaceMeta = null;
            }

            if ( interfaceMeta == null) {
                /* set */
                try {
                    forwardInterface.dynamicVariable(MetaPragma.metaKey, forwardMeta);
                } catch(Exception e){};
            }
            else if (interfaceMeta != forwardMeta) {
                 /* The above check is needed because sometimes
                 a forward entry is processed more the once.
                 Not sure why */
                /* merge */
                for (int i=0; i < forwardMeta.size(); i++){
                    try {
                        Object obj = forwardMeta.elementAt(i);
                        interfaceMeta.addElement(obj);
                    } catch (Exception e){};
                }
            }
         }
    }

    /**
     * parse pragma message and place into vector v.
     * @param v: vector to add message
     * @param msg: string of comma separated message, perhaps with comment.
     * This is implemented as a state machine as follows:
     *
     *  State          token        next             action
     *  -----------------------------------------------------
     *   initial     whitespace     initial          
     *   initial     SlashStar      comment          
     *   initial     SlashSlash     final              
     *   initial     no more        final              
     *   initial     text           text             add to text buffer
     *   initial     StarSlash      initial
     *   comment     StarSlash      initial          
     *   comment     SlashStar      comment
     *   comment     whitespace     comment
     *   comment     SlashSlash     comment          
     *   comment     text           comment
     *   comment     no more        final
     *   text        text           text              add to buffer
     *   text        SlashStar      comment           put in vector
     *   text        whitespace     initial           put in vector
     *   text        SlashSlash     final             put in vector
     *   text        StarSlash      initial           put in vector
     *   text        no more        final             put in vector
     *   
    */
    private static int initialState = 0;
    private static int commentState = 1;
    private static int textState = 2;
    private static int finalState =3;

    private void parseMsg(Vector v, String msg){
        int state = initialState;
        String text = "";
        int index = 0;
        while ( state != finalState ){
             boolean isNoMore = index >= msg.length();
             char ch = ' ';   
             boolean isSlashStar = false;
             boolean isSlashSlash = false;
             boolean isWhiteSpace = false;
             boolean isStarSlash = false;
             boolean isText = false;
             if (!isNoMore ){
                 ch = msg.charAt(index);
                 if (ch == '/' && index+1 < msg.length()){
                     if (msg.charAt(index+1) == '/'){
                         isSlashSlash = true;
                          index++;
                     }
                     else if (msg.charAt(index+1) == '*'){
                         isSlashStar= true;
                         index++;
                     } else isText = true;
                 }
                 else if (ch == '*' && index+1 < msg.length() ){
                     if (msg.charAt(index+1) == '/'){
                         isStarSlash = true;
                         index++;
                     } else isText = true;
                 } 
                 else if ( Character.isSpace(ch) || (ch == ',') // 59601
                              || (ch == ';') ) // 59683
                     isWhiteSpace = true;
                 else isText = true;
            }
   
            if (state == initialState){
                   if (isSlashStar){
                      state = commentState;
                   }
                   else if (isSlashSlash || isNoMore){
                      state = finalState;
                   }
                   else if (isText){
                       state = textState;
                       text = text+ ch;
                   }
             }
             else if (state == commentState){
                   if (isNoMore){
                        state = finalState;
                   }
                   else if ( isStarSlash){
                        state = initialState;
                   }
             }
             else if (state == textState){
                   if (isNoMore || isStarSlash || isSlashSlash ||
                       isSlashStar || isWhiteSpace ){
                       if (!text.equals("")) {
                           v.addElement(text);
// System.err.println("adding " + text);
                           text = "";
                       }
                       if (isNoMore)
                            state = finalState;
                       else if (isSlashStar)
                            state = commentState;
                       else state = initialState;
                   }
                   else if (isText){
                       text = text+ch;
                   }
             }
             index++;
        }
    }

}
