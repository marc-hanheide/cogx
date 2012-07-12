// Author: Marko Mahnič
// Created: 2012-06-15

#ifndef _DIALOG_DIALOGINTERACTION_HPP_4FDEDBD6_
#define _DIALOG_DIALOGINTERACTION_HPP_4FDEDBD6_

#include <cast/core.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <string>
#include <map>

namespace v11n
{
class CDialogInteraction : public cast::ManagedComponent
{
  std::map<std::string, std::string> mOptions;
  std::string mPresetFile;
  
public:
  CDialogInteraction();

  virtual ~CDialogInteraction() {}
  
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  using CASTComponent::sleepComponent;

#ifdef FEAT_VISUALIZATION
private:
  friend class CDisplayClient;
  class CDisplayClient: public cogx::display::CDisplayClient
  {
  public:
    CDialogInteraction* mpDlgInt;
    std::string mDialogId;
    CDisplayClient(CDialogInteraction* pDlgInt);
    void onDialogValueChanged(const std::string& dialogId,
        const std::string& name, const std::string& value); /*override*/
    void handleDialogCommand(const std::string& dialogId,
        const std::string& command, const std::string& params); /*override*/

    void addSpokenText(const std::string& text);
  };
  CDisplayClient mDisplay;
  CDisplayClient& display()
  {
    return mDisplay;
  }
#endif
  void loadPresets();
  void sayText(const std::string& text);
  void onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_PhonString(const cast::cdl::WorkingMemoryChange & _wmc);
};

}
#endif /* _DIALOG_DIALOGINTERACTION_HPP_4FDEDBD6_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
