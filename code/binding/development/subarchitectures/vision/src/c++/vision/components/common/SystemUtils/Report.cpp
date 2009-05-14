/** @file Report.cpp
 *  @brief Configurator of report messages.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "Report.h"
#include <cstdarg>

Report::Report()
    :m_bShow(false), m_bDebug(true), m_bDebug5(false), m_bGraphic(false)
{
}

Report::~Report()
{
}

