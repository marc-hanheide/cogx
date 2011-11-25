/* Generated By:JavaCC: Do not edit this line. BNConfigParserConstants.java */
/** New line translator. */

package binder.bayesiannetwork.configparser;


/**
 * Token literal values and constants.
 * Generated by org.javacc.parser.OtherFilesGen#start()
 */
public interface BNConfigParserConstants {

  /** End of File. */
  int EOF = 0;
  /** RegularExpression Id. */
  int NODEID = 5;
  /** RegularExpression Id. */
  int EDGEID = 6;
  /** RegularExpression Id. */
  int FEATVALUESID = 7;
  /** RegularExpression Id. */
  int FEATPROBSID = 8;
  /** RegularExpression Id. */
  int ID = 9;
  /** RegularExpression Id. */
  int NUM = 10;
  /** RegularExpression Id. */
  int LBRACE = 11;
  /** RegularExpression Id. */
  int RBRACE = 12;
  /** RegularExpression Id. */
  int PROB = 13;

  /** Lexical state. */
  int DEFAULT = 0;

  /** Literal token values. */
  String[] tokenImage = {
    "<EOF>",
    "\" \"",
    "\"\\t\"",
    "\"\\n\"",
    "\"\\r\"",
    "\"nodes\"",
    "\"edges\"",
    "\"featurevalues\"",
    "\"featureprobs\"",
    "<ID>",
    "<NUM>",
    "\"{\"",
    "\"}\"",
    "<PROB>",
    "\",\"",
    "\"P(\"",
    "\"=\"",
    "\")\"",
    "\"|\"",
    "\"->\"",
  };

}