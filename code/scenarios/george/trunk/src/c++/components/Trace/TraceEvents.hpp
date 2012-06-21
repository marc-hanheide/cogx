// Author: Marko Mahnič
// Created: 2012-06-20

#ifndef _TRACE_TRACEEVENTS_HPP_4FE19FB9_
#define _TRACE_TRACEEVENTS_HPP_4FE19FB9_

#include <cast/core.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <VisionData.hpp>

#include <string>
#include <map>

namespace v11n
{
class CTraceEvents : public cast::ManagedComponent
{
  std::map<std::string, std::string> mOptions;
  std::map<std::string, long> mCount;
  std::map<cast::cdl::WorkingMemoryAddress, VisionData::VisualObjectPtr> mVisualObjects;
  
public:
  CTraceEvents();

  virtual ~CTraceEvents() {}
  
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
    long long lineId;
  public:
    CTraceEvents* mpTrace;
    std::string mDialogId;
    CDisplayClient(CTraceEvents* pDlgInt);
    void addSpeech(const std::string& text, const std::string& info="");
    void addCounts(const std::string& event, const std::string& text, const std::string& info="");
    void addWmEntry(const std::string& text, const std::string& info="");
  private:
    void sendTraceLine(const std::string& ta, const std::string& tb, const std::string& tc, const std::string& info);
  };
  CDisplayClient mDisplay;
  CDisplayClient& display()
  {
    return mDisplay;
  }
#endif
  std::string getObjectCountStr();
  long getVisibleVisualObjectCount();
  void onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_PhonString(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onDel_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_SOI(const cast::cdl::WorkingMemoryChange & _wmc);
  void onDel_SOI(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_LearnerRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_LearnerRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_MoveToVcCommand(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_MoveToVcCommand(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_AnalyzeProtoObjectCommand(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_AnalyzeProtoObjectCommand(const cast::cdl::WorkingMemoryChange & _wmc);
  void onAdd_Motive(const cast::cdl::WorkingMemoryChange & _wmc);
  void onChange_Motive(const cast::cdl::WorkingMemoryChange & _wmc);
};

}
#endif /* _TRACE_TRACEEVENTS_HPP_4FE19FB9_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
