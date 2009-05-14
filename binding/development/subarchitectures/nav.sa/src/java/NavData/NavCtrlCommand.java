package NavData;


/**
* NavData/NavCtrlCommand.java .
* Generated by the IDL-to-Java compiler (portable), version "3.2"
* from NavData.idl
* Freitag, 13. M�rz 2009 17.31 Uhr CET
*/

public final class NavCtrlCommand implements org.omg.CORBA.portable.IDLEntity
{
  public NavData.CtrlCommand m_command = null;
  public double m_x = (double)0;
  public double m_y = (double)0;
  public double m_r = (double)0;
  public double m_theta = (double)0;
  public double m_distance = (double)0;
  public int m_areaID = (int)0;
  public int m_nodeID = (int)0;
  public NavData.StatusError m_status = null;
  public NavData.Completion m_completion = null;

  public NavCtrlCommand ()
  {
  } // ctor

  public NavCtrlCommand (NavData.CtrlCommand _m_command, double _m_x, double _m_y, double _m_r, double _m_theta, double _m_distance, int _m_areaID, int _m_nodeID, NavData.StatusError _m_status, NavData.Completion _m_completion)
  {
    m_command = _m_command;
    m_x = _m_x;
    m_y = _m_y;
    m_r = _m_r;
    m_theta = _m_theta;
    m_distance = _m_distance;
    m_areaID = _m_areaID;
    m_nodeID = _m_nodeID;
    m_status = _m_status;
    m_completion = _m_completion;
  } // ctor

} // class NavCtrlCommand
