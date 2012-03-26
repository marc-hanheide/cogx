/**
 * Generic parts of a PTZ server. Copied from Michael Zillich's video
 * server. Currently all control is position-based.
 * 
 * Display Server integration by Marko Mahnič
 */

#include <cast/core.hpp>
#include <autogen/PTZ.hpp>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace ptz {

  //fwd decl of main server class
  class PTZServer;

  /**
   * Ice interface for PTZ functionality.
   */
  class PTZServerI : public ptz::PTZInterface {
  private:
    PTZServer * m_ptzServer;
  public:
    PTZServerI(PTZServer *_ptzServer) : m_ptzServer(_ptzServer) {}    

    virtual ptz::PTZReading getPose(const Ice::Current & _crt) const;

    virtual void setPose(const ptz::PTZPose & _pose, 
			 const Ice::Current & _crt);

  };



  /**
   * Server to provide access to a pan-tilt(-zoom) server.
   */
  class PTZServer :
     virtual public cast::CASTComponent
   {
    
  public:
    PTZServer();

    virtual ~PTZServer() {}
    
    virtual void configure(const std::map<std::string,std::string> & _config);
    virtual void start();
    virtual void runComponent();

  protected:

    friend class PTZServerI;

    //methods all ptz servers must implement

    /**
     * Get the current ptz pose.
     */  
    virtual PTZReading getPose() = 0;

    /**
     * Set the desired ptz pose.
     */
    virtual void setPose(const PTZPose & _pose) = 0;

#ifdef FEAT_VISUALIZATION
private:
  bool mbPoseWasSet;
  void sendPtuStateToDialog();

  friend class CDisplayClient;
  class CDisplayClient: public cogx::display::CDisplayClient
  {
  public:
    PTZServer* mpPtzServer;
    std::string mDialogId;
    CDisplayClient(PTZServer* pPtzServer);
    void onDialogValueChanged(const std::string& dialogId,
        const std::string& name, const std::string& value); /*override*/
    void handleDialogCommand(const std::string& dialogId,
        const std::string& command, const std::string& params); /*override*/
  };
  CDisplayClient mDisplay;
  CDisplayClient& display()
  {
    return mDisplay;
  }
#endif
  };

}
