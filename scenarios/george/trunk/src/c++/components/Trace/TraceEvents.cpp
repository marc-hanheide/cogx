// Author: Marko Mahnič
// Created: 2012-06-20

#include "TraceEvents.hpp"
#include <Timers.hpp>

#include <dialogue.hpp>
#include <dialogue_utils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <motivation.hpp>

#include <sstream>
#include <cmath>

using namespace cast;
namespace dlgice = de::dfki::lt::tr::dialogue::slice;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new v11n::CTraceEvents();
  }
}

namespace v11n
{

static void replaceAll(std::string& str, const std::string& from, const std::string& to)
{
  size_t start_pos = 0;
  while((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}


CTraceEvents::CTraceEvents()
#ifdef FEAT_VISUALIZATION
  : mDisplay(this)
#endif
  {
    mOptions["dialogue.sa"] = "dialogue"; // hardcoded default SA name; change with --dialogue-sa
  }

void CTraceEvents::configure(const std::map<std::string,std::string> & _config)
{
  std::map<std::string,std::string>::const_iterator it;

  // CONFIG: --dialogue-sa
  // TYPE: string (subarchitecture-id)
  // DEFAULT: "dialogue"
  // The subarchitecture id in which ASR is running
  if((it = _config.find("--dialogue-sa")) != _config.end())
  {
    if (it->second == "") mOptions["dialogue.sa"] = getSubarchitectureID();
    else mOptions["dialogue.sa"] = it->second;
  }

#ifdef FEAT_VISUALIZATION
  display().configureDisplayClient(_config);
#endif
}

void CTraceEvents::start()
{
  // Speech
  addChangeFilter(
      cast::createGlobalTypeFilter<dlgice::synthesize::SpokenOutputItem>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onAdd_SpokenItem)
      );

  addChangeFilter(
      cast::createGlobalTypeFilter<dlgice::asr::PhonString>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onAdd_PhonString)
      );

  // VisualObject
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::VisualObject>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onAdd_VisualObject)
      );
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::VisualObject>(cast::cdl::DELETE),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onDel_VisualObject)
      );
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::VisualObject>(cast::cdl::OVERWRITE),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onChange_VisualObject)
      );

  // ProtoObject
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::ProtoObject>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onAdd_ProtoObject)
      );
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::ProtoObject>(cast::cdl::DELETE),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onDel_ProtoObject)
      );

  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::SOI>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onAdd_SOI)
      );
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::SOI>(cast::cdl::DELETE),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onDel_SOI)
      );

  // VisualLearningTask
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::VisualLearningTask>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onAdd_LearningTask)
      );
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::VisualLearningTask>(cast::cdl::OVERWRITE),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onChange_LearningTask)
      );

  // VisualLearnerRecognitionTask
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::VisualLearnerRecognitionTask>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onAdd_LearnerRecognitionTask)
      );
  addChangeFilter(
      cast::createGlobalTypeFilter<VisionData::VisualLearnerRecognitionTask>(cast::cdl::OVERWRITE),
      new cast::MemberFunctionChangeReceiver<CTraceEvents>(
        this, &CTraceEvents::onChange_LearnerRecognitionTask)
      );

  // MoveToViewConeCommand
  addChangeFilter(createGlobalTypeFilter<VisionData::MoveToViewConeCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTraceEvents>(this,
        &CTraceEvents::onAdd_MoveToVcCommand));

  addChangeFilter(createGlobalTypeFilter<VisionData::MoveToViewConeCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTraceEvents>(this,
        &CTraceEvents::onChange_MoveToVcCommand));

  // AnalyzeProtoObjectCommand
  addChangeFilter(createGlobalTypeFilter<VisionData::AnalyzeProtoObjectCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTraceEvents>(this,
        &CTraceEvents::onAdd_AnalyzeProtoObjectCommand));

  addChangeFilter(createGlobalTypeFilter<VisionData::AnalyzeProtoObjectCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTraceEvents>(this,
        &CTraceEvents::onChange_AnalyzeProtoObjectCommand));

  // motivaion.slice.Motive
  addChangeFilter(createGlobalTypeFilter<motivation::slice::Motive>(cdl::ADD),
      new MemberFunctionChangeReceiver<CTraceEvents>(this,
        &CTraceEvents::onAdd_Motive));

  addChangeFilter(createGlobalTypeFilter<motivation::slice::Motive>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<CTraceEvents>(this,
        &CTraceEvents::onChange_Motive));

#ifdef FEAT_VISUALIZATION
  display().connectIceClient(*this);
  //display().mDialogId = getComponentID() + "#Dialogue";
  //display().installEventReceiver();
  //display().addDialog(display().mDialogId, res_dialogue_ui, res_dialogue_js, "DialogueInteraction dlgctrl");

  display().setHtml("GeorgeTrace", "100", "<table border='1' style='border-collapse:collapse;'>");
  display().setHtml("GeorgeTrace", "199", "</table>");
#endif
}

void CTraceEvents::runComponent()
{
  const int intervalMs = 100;  // desired time between checks
  castutils::CCastPaceMaker<CTraceEvents> pace(*this, intervalMs, 2);

  while (isRunning()) {
    pace.sync();
  }
}

long CTraceEvents::getVisibleVisualObjectCount()
  // Enter: Locked mVisualObjects
{
  long count = 0;
  for(auto v : mVisualObjects) {
    if (! v.second.get()) continue;
    if (v.second->presence == VisionData::VopVISIBLE)
      ++count;
  }
  return count;
}

std::string CTraceEvents::getObjectCountStr()
{
  std::ostringstream ss;
  ss << "SOI:" << mCount["SOI"];
  ss << ", PO:" << mCount["ProtoObject"];
  ss << ", VO:" << mCount["VisualObject-v"] << "/" << mCount["VisualObject"];
  return ss.str();
}

// The robot creates SpokenOutputItem-s -> for TTS
void CTraceEvents::onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc)
{
#ifdef FEAT_VISUALIZATION
  dlgice::synthesize::SpokenOutputItemPtr psaid =
    getMemoryEntry<dlgice::synthesize::SpokenOutputItem>(_wmc.address);
  display().addSpeech("R: " + psaid->phonString);
#endif
}

// The human creates PhonString-s with sayText
void CTraceEvents::onAdd_PhonString(const cast::cdl::WorkingMemoryChange & _wmc)
{
#ifdef FEAT_VISUALIZATION
  dlgice::asr::PhonStringPtr psaid =
    getMemoryEntry<dlgice::asr::PhonString>(_wmc.address);
  display().addSpeech("H: " + psaid->wordSequence);
#endif
}

void CTraceEvents::onAdd_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  auto pvo = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mVisualObjects[_wmc.address] = pvo;
    mCount["VisualObject"] += 1;
    mCount["VisualObject-v"] = getVisibleVisualObjectCount();
    display().addCounts("+VO", getObjectCountStr());
  }
}

void CTraceEvents::onDel_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mVisualObjects[_wmc.address] = VisionData::VisualObjectPtr();
    mCount["VisualObject"] -= 1;
    mCount["VisualObject-v"] = getVisibleVisualObjectCount();
    display().addCounts("-VO", getObjectCountStr());
  }
}

void CTraceEvents::onChange_VisualObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  auto pvo = getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mVisualObjects[_wmc.address] = pvo;
    mCount["VisualObject-v"] = getVisibleVisualObjectCount();
    display().addCounts("(VO)", getObjectCountStr());
  }
}

void CTraceEvents::onAdd_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["ProtoObject"] += 1;
    display().addCounts("+PO", getObjectCountStr());
  }
}

void CTraceEvents::onDel_ProtoObject(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["ProtoObject"] -= 1;
    display().addCounts("-PO", getObjectCountStr());
  }
}

void CTraceEvents::onAdd_SOI(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["SOI"] += 1;
    display().addCounts("+SOI", getObjectCountStr());
  }
}

void CTraceEvents::onDel_SOI(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["SOI"] -= 1;
    display().addCounts("-SOI", getObjectCountStr());
  }
}

void CTraceEvents::onAdd_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["VisualLearningTask-add"] += 1;
    display().addWmEntry("ADD VisualLearningTask");
  }
}

void CTraceEvents::onChange_LearningTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["VisualLearningTask-done"] += 1;
    display().addWmEntry("DONE VisualLearningTask");
  }
}

void CTraceEvents::onAdd_LearnerRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["VisualLearnerRecognitionTask-add"] += 1;
    display().addWmEntry("ADD VisualLearnerRecognitionTask");
  }
}

void CTraceEvents::onChange_LearnerRecognitionTask(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    mCount["VisualLearnerRecognitionTask-done"] += 1;
    display().addWmEntry("DONE VisualLearnerRecognitionTask");
  }
}

void CTraceEvents::onAdd_MoveToVcCommand(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    //mCount["VisualLearnerRecognitionTask-add"] += 1;
    display().addWmEntry("ADD MoveToViewConeCommand");
  }
}

void CTraceEvents::onChange_MoveToVcCommand(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    //mCount["VisualLearnerRecognitionTask-add"] += 1;
    display().addWmEntry("DONE MoveToViewConeCommand");
  }
}

void CTraceEvents::onAdd_AnalyzeProtoObjectCommand(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    //mCount["VisualLearnerRecognitionTask-add"] += 1;
    display().addWmEntry("ADD AnalyzeProtoObjectCommand");
  }
}

void CTraceEvents::onChange_AnalyzeProtoObjectCommand(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    //mCount["VisualLearnerRecognitionTask-add"] += 1;
    display().addWmEntry("DONE AnalyzeProtoObjectCommand");
  }
}

std::string motiveStatusStr(motivation::slice::MotiveStatus st)
{
  //char buf[16];
  //sprintf(buf, "%d", (int)st);
  //return buf;
  std::string s;
#define EN_PREFIX motivation::slice
#define EN_CASE(X) case EN_PREFIX::X: s = #X; break
  switch(st) {
    EN_CASE(UNSURFACED);
    EN_CASE(SURFACED);
    EN_CASE(POSSIBLE);
    EN_CASE(IMPOSSIBLE);
    EN_CASE(ACTIVE);
    EN_CASE(COMPLETED);
    EN_CASE(WILDCARD);
    default: s = "?"; break;
  }
#undef EN_CASE
#undef EN_PREFIX
  return s;
}

void CTraceEvents::onAdd_Motive(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    //mCount["VisualLearnerRecognitionTask-add"] += 1;
    auto pmo = getMemoryEntry<motivation::slice::Motive>(_wmc.address);
    display().addWmEntry("ADD " + pmo->ice_id().substr(21) 
        + " " + motiveStatusStr(pmo->status));
    // TODO: ::autogen::Planner::GoalPtr goal;
  }
}

void CTraceEvents::onChange_Motive(const cast::cdl::WorkingMemoryChange & _wmc)
{
  {
    //std::lock_guard<std::mutex> lock(mWmCopyMutex);
    //mCount["VisualLearnerRecognitionTask-add"] += 1;
    auto pmo = getMemoryEntry<motivation::slice::Motive>(_wmc.address);
    display().addWmEntry("CHG " + pmo->ice_id().substr(21)
        + " " + motiveStatusStr(pmo->status));
    // TODO: if SURFACED: ::autogen::Planner::GoalPtr goal;
  }
}

#ifdef FEAT_VISUALIZATION

CTraceEvents::CDisplayClient::CDisplayClient(CTraceEvents* pTrace)
  : mpTrace(pTrace)
{
  lineId = 0;
}

void CTraceEvents::CDisplayClient::addSpeech(const std::string& text)
{
  sendTraceLine(text, "", "");
}

void CTraceEvents::CDisplayClient::addCounts(const std::string& event, const std::string& text)
{
  sendTraceLine("", event, text);
}

void CTraceEvents::CDisplayClient::addWmEntry(const std::string& text)
{
  sendTraceLine("", text, "");
}

std::string mkseqid(const std::string& prefix, long id, int dir=1)
{
  std::ostringstream ss;
  if (dir >= 0) ss << id;
  else ss << (LONG_MAX - id);
  std::string sid = ss.str();

  while (sid.length() <= 4) sid = "0000" + sid;
  while (sid.length() <= 6) sid = "00" + sid;
  while (sid.length() < 8)  sid = "0" + sid;

  return prefix + sid;
}

void CTraceEvents::CDisplayClient::sendTraceLine(const std::string& ta, const std::string& tb, const std::string& tc)
{
  cast::cdl::CASTTime tm = mpTrace->getCASTTime();
  double now = tm.s + 1e-6 * tm.us;
  char buf[16];
  if (now > 1e9 || now < 0) now = -1;
  sprintf(buf, "%.3f", now);
  std::ostringstream ss;
  ss << "<tr><td>" << buf << "</td>";
  ss << "<td>" << ta << "</td><td>" << tb << "</td><td>" << tc << "</td></tr>";

  ++lineId;
  setHtml("GeorgeTrace", mkseqid("110-", lineId), ss.str());
}


#endif
}
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
