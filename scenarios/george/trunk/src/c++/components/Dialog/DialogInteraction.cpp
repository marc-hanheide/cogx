// Author: Marko Mahnič
// Created: 2012-06-15

#include "DialogInteraction.hpp"
#include "StringFmt.hpp"
#include <Timers.hpp>

#include <dialogue.hpp>
#include <dialogue_utils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

using namespace cast;
namespace dlgice = de::dfki::lt::tr::dialogue::slice;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new v11n::CDialogInteraction();
  }
}

namespace v11n
{

#ifdef FEAT_VISUALIZATION
#include "res/dialogue_gen.inc"
#endif

CDialogInteraction::CDialogInteraction()
#ifdef FEAT_VISUALIZATION
  : mDisplay(this)
#endif
{
  mOptions["dialogue.sa"] = "dialogue"; // hardcoded default SA name; change with --dialogue-sa
}

void CDialogInteraction::configure(const std::map<std::string,std::string> & _config)
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

  if((it = _config.find("--presets")) != _config.end())
  {
    mPresetFile = it->second;
  }

#ifdef FEAT_VISUALIZATION
  display().configureDisplayClient(_config);
#endif
}

void CDialogInteraction::start()
{
  addChangeFilter(
      cast::createGlobalTypeFilter<dlgice::synthesize::SpokenOutputItem>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CDialogInteraction>(
        this, &CDialogInteraction::onAdd_SpokenItem)
      );

  addChangeFilter(
      cast::createGlobalTypeFilter<dlgice::asr::PhonString>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<CDialogInteraction>(
        this, &CDialogInteraction::onAdd_PhonString)
      );

#ifdef FEAT_VISUALIZATION
  display().connectIceClient(*this);
  display().installEventReceiver();
  display().mDialogId = getComponentID() + "#Dialogue";
  display().addDialog(display().mDialogId, res_dialogue_ui, res_dialogue_js, "DialogueInteraction dlgctrl");
#endif
}


void CDialogInteraction::loadPresets()
{
  std::ifstream f;
  f.open(mPresetFile.c_str());
  if (f.fail()) {
    error("File not found: '" + mPresetFile + "'");
    return;
  }

  std::vector<std::string> validSections({
      "question", "assertion", "instruction", "color", "shape", "type"
      });
  std::string section;
  std::vector<std::string> items;

  auto emitSection = [&]() -> void
  {
    if (items.size() < 1) {
      println("no items in section %s", section.c_str());
      return;
    }
    if (std::find(validSections.begin(), validSections.end(), section) == validSections.end()) {
      println("invalid section %s", section.c_str());
      return;
    }

    std::ostringstream ss;
    ss << "dlgctrl.setPresets('" << section << "', [";
    int cnt = 0;
    for (auto sitem: items) {
      _s_::replace(sitem, "\'", "");
      _s_::replace(sitem, "\"", "");
      _s_::replace(sitem, "\\", "");
      if (cnt++ > 0) ss << ", ";
      ss << "'" << sitem << "'";
    }
    ss << "]);";
#ifdef FEAT_VISUALIZATION
    println(" *** %s", ss.str().c_str());
    display().execInDialog(display().mDialogId, ss.str());
#endif
  };

  while (f.good() && !f.eof()) {
    std::string line;
    std::getline(f, line);
    line = _s_::strip(line);
    if (line.size() < 1 || _s_::startswith(line, "#")) {
      continue;
    }
    if (_s_::startswith(line, "[") && _s_::endswith(line, "]")) {
      if (section != "" && items.size() > 0) {
        emitSection();
      }
      section = _s_::strip(line, "[]\t ");
      items.clear();
      continue;
    }
    items.push_back(line);
  }
  if (section != "" && items.size() > 0) {
    emitSection();
  }
}

void CDialogInteraction::runComponent()
{
  const int intervalMs = 100;  // desired time between checks
  castutils::CCastPaceMaker<CDialogInteraction> pace(*this, intervalMs, 2);

#ifdef FEAT_VISUALIZATION
  display().execInDialog(display().mDialogId, "dlgctrl.clearSpokenText();");

  if (mPresetFile != "")
    loadPresets();
#endif

  while (isRunning()) {
    pace.sync();
  }
}

// The robot creates SpokenOutputItem-s -> for TTS
void CDialogInteraction::onAdd_SpokenItem(const cast::cdl::WorkingMemoryChange & _wmc)
{
#ifdef FEAT_VISUALIZATION
  dlgice::synthesize::SpokenOutputItemPtr psaid =
    getMemoryEntry<dlgice::synthesize::SpokenOutputItem>(_wmc.address);
  display().addSpokenText("R: " + psaid->phonString);
#endif
}

// The human creates PhonString-s with sayText
void CDialogInteraction::onAdd_PhonString(const cast::cdl::WorkingMemoryChange & _wmc)
{
#ifdef FEAT_VISUALIZATION
  dlgice::asr::PhonStringPtr psaid =
    getMemoryEntry<dlgice::asr::PhonString>(_wmc.address);
  display().addSpokenText("H: " + psaid->wordSequence);
#endif
}

void CDialogInteraction::sayText(const std::string& text)
{
  cast::cdl::WorkingMemoryAddress addr;
  addr.subarchitecture = mOptions["dialogue.sa"];
  addr.id = newDataID();

  dlgice::asr::PhonStringPtr sayWhat = dlgice::asr::newPhonString(text);
  if (! sayWhat.get()) {
    error("Could not allocate PhonString");
    return;
  }
  sayWhat->id = addr.id;

  long now = getCASTTime().s * 1000;
  sayWhat->ival->begin->msec = now;
  sayWhat->ival->end->msec = now + 500;
  log("Saying: '%s'.", text.c_str());
  //display().addSpokenText("(dbg) H: " + text);

  addToWorkingMemory(addr, sayWhat);
}

#ifdef FEAT_VISUALIZATION

CDialogInteraction::CDisplayClient::CDisplayClient(CDialogInteraction* pDlgInt)
  : mpDlgInt(pDlgInt)
{
}

void CDialogInteraction::CDisplayClient::onDialogValueChanged(const std::string& dialogId,
    const std::string& name, const std::string& value)
{
  if (dialogId == mDialogId) {
    if (name == "HumanSaid") {
      //println(" *** HumanSaid command from dialog *** " + value);
      mpDlgInt->sayText(value);
    }
  }
}

void CDialogInteraction::CDisplayClient::handleDialogCommand(const std::string& dialogId,
      const std::string& command, const std::string& params)
{
  if (dialogId == mDialogId) {
#if 0
    //println(" *** handleDialogCommand *** " + command);
    if (command == "sendStateToDialog")
      mpPtzServer->sendPtuPositionToDialog(/*force=*/ true);
#endif
  }
}

static void replaceAll(std::string& str, const std::string& from, const std::string& to)
{
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();
    }
}

void CDialogInteraction::CDisplayClient::addSpokenText(const std::string& text)
{
  std::string t = text;
  replaceAll(t, "\\", "\\\\");
  replaceAll(t, "'", "\\'");
  replaceAll(t, "\"", "\\\"");
  replaceAll(t, "\n", "\\n");
  replaceAll(t, "\r", "\\r");
  std::ostringstream ss;
  ss << "dlgctrl.addSpokenText('";
  ss << t;
  ss << "');";
  execInDialog(mDialogId, ss.str());
}

#endif
}
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
