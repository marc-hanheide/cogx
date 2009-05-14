package ComaData;


/**
* ComaData/ComaReasonerFunctionType.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from ComaData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/


/// These are the coma reasoner function types
public class ComaReasonerFunctionType implements org.omg.CORBA.portable.IDLEntity
{
  private        int __value;
  private static int __size = 20;
  private static ComaData.ComaReasonerFunctionType[] __array = new ComaData.ComaReasonerFunctionType [__size];

  public static final int _GetAllDirectConcepts = 0;
  public static final ComaData.ComaReasonerFunctionType GetAllDirectConcepts = new ComaData.ComaReasonerFunctionType(_GetAllDirectConcepts);
  public static final int _GetAllConcepts = 1;
  public static final ComaData.ComaReasonerFunctionType GetAllConcepts = new ComaData.ComaReasonerFunctionType(_GetAllConcepts);
  public static final int _GetMostSpecificConcepts = 2;
  public static final ComaData.ComaReasonerFunctionType GetMostSpecificConcepts = new ComaData.ComaReasonerFunctionType(_GetMostSpecificConcepts);
  public static final int _GetBasicLevelConcepts = 3;
  public static final ComaData.ComaReasonerFunctionType GetBasicLevelConcepts = new ComaData.ComaReasonerFunctionType(_GetBasicLevelConcepts);
  public static final int _GetAllInstances = 4;
  public static final ComaData.ComaReasonerFunctionType GetAllInstances = new ComaData.ComaReasonerFunctionType(_GetAllInstances);
  public static final int _GetRelatedInstances = 5;
  public static final ComaData.ComaReasonerFunctionType GetRelatedInstances = new ComaData.ComaReasonerFunctionType(_GetRelatedInstances);
  public static final int _GetInstancesByName = 6;
  public static final ComaData.ComaReasonerFunctionType GetInstancesByName = new ComaData.ComaReasonerFunctionType(_GetInstancesByName);
  public static final int _AreConsEquivalent = 7;
  public static final ComaData.ComaReasonerFunctionType AreConsEquivalent = new ComaData.ComaReasonerFunctionType(_AreConsEquivalent);
  public static final int _IsConSupercon = 8;
  public static final ComaData.ComaReasonerFunctionType IsConSupercon = new ComaData.ComaReasonerFunctionType(_IsConSupercon);
  public static final int _IsConSubcon = 9;
  public static final ComaData.ComaReasonerFunctionType IsConSubcon = new ComaData.ComaReasonerFunctionType(_IsConSubcon);
  public static final int _CompareCons = 10;
  public static final ComaData.ComaReasonerFunctionType CompareCons = new ComaData.ComaReasonerFunctionType(_CompareCons);
  public static final int _GetObjectMobility = 11;
  public static final ComaData.ComaReasonerFunctionType GetObjectMobility = new ComaData.ComaReasonerFunctionType(_GetObjectMobility);
  public static final int _GetTypicalObjects = 12;
  public static final ComaData.ComaReasonerFunctionType GetTypicalObjects = new ComaData.ComaReasonerFunctionType(_GetTypicalObjects);
  public static final int _IsInstanceOf = 13;
  public static final ComaData.ComaReasonerFunctionType IsInstanceOf = new ComaData.ComaReasonerFunctionType(_IsInstanceOf);
  public static final int _AddInstance = 14;
  public static final ComaData.ComaReasonerFunctionType AddInstance = new ComaData.ComaReasonerFunctionType(_AddInstance);
  public static final int _DeleteInstance = 15;
  public static final ComaData.ComaReasonerFunctionType DeleteInstance = new ComaData.ComaReasonerFunctionType(_DeleteInstance);
  public static final int _AddInstanceName = 16;
  public static final ComaData.ComaReasonerFunctionType AddInstanceName = new ComaData.ComaReasonerFunctionType(_AddInstanceName);
  public static final int _AddInstanceNumberTag = 17;
  public static final ComaData.ComaReasonerFunctionType AddInstanceNumberTag = new ComaData.ComaReasonerFunctionType(_AddInstanceNumberTag);
  public static final int _AddRelation = 18;
  public static final ComaData.ComaReasonerFunctionType AddRelation = new ComaData.ComaReasonerFunctionType(_AddRelation);
  public static final int _GenerateRefEx = 19;
  public static final ComaData.ComaReasonerFunctionType GenerateRefEx = new ComaData.ComaReasonerFunctionType(_GenerateRefEx);

  public int value ()
  {
    return __value;
  }

  public static ComaData.ComaReasonerFunctionType from_int (int value)
  {
    if (value >= 0 && value < __size)
      return __array[value];
    else
      throw new org.omg.CORBA.BAD_PARAM ();
  }

  protected ComaReasonerFunctionType (int value)
  {
    __value = value;
    __array[__value] = this;
  }
} // class ComaReasonerFunctionType
