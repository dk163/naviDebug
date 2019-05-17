/******************************************************************************
 *
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice is
 * included in all copies of any software which is or includes a copy or
 * modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
 * THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: Android GNSS Driver
 *
 ******************************************************************************
 * $Id$
 * $HeadURL$
 *****************************************************************************/

#include <algorithm>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <memory>

#include "ubx_cfg.h"
#include "ubx_log.h"

namespace
{
  bool isBooleanString(const std::string &number) { return number.substr(0, 2).compare("0b") == 0; }
  int getBoolean(const std::string &number) { return std::stoi(number.substr(2), nullptr, 2); }
  int getInt(const std::string &number) { return std::stoi(number, nullptr, 0); }

  int getValue(const std::string &number)
  {
    if (isBooleanString(number))
    {
      return getBoolean(number);
    }
    else
    {
      return getInt(number);
    }
  }
  std::string getVendorPath(const char* fileName) {
    std::string vendorFile{fileName};
    size_t pos = vendorFile.find("system");
    if(pos != std::string::npos) {
    vendorFile.replace(pos,6,"vendor");
    }
    return vendorFile;
  }
} // anonymous namespace
CCfg::CCfg() : configuration{} {}

CCfg::~CCfg() {}

void CCfg::load(const char *fileName)
{
  if(!fileName) {
    UBX_LOG(LCAT_VERBOSE, "Filename pointer invalid");
    return;
  }
  //std::ifstream file(fileName, std::ios::in);
  auto file = std::make_unique<std::ifstream>(fileName, std::ios::in);
  /* UBX_LOG(LCAT_DEBUG, "fileName=\"%s\"", fileName); */
  if (!file->is_open())
  {
    //try vendor path
    file = std::make_unique<std::ifstream>(getVendorPath(fileName).c_str(), std::ios::in);
    if (!file->is_open())
    {
      // failed
      UBX_LOG(LCAT_VERBOSE, "Can not open '%s' file : %i", fileName, errno);
      return;
    }
  }

  std::string line;
  while (std::getline(*file, line))
  {
    readConfigLine(line);
  }
  file->close();
}

void CCfg::load(const char *configuration, int32_t /* length*/)
{
  std::string line;
  std::stringstream stream;
  stream << configuration;
  while (std::getline(stream, line))
  {
    readConfigLine(line);
  }
}

int CCfg::get(const char *item, int def) const
{
  int retVal = def;
  auto compareFunction = [item](const std::pair<std::string, std::string> &elem) {
    return elem.first.compare(std::string(item)) == 0;
  };

  if (item == nullptr)
  {
    return retVal;
  }

  auto it = std::find_if(configuration.begin(), configuration.end(), compareFunction);

  if (it != configuration.end())
  {
    retVal = getValue(it->second);
  }

  /* UBX_LOG(LCAT_DEBUG, "item=\"%s\" def=%d => %d", item, def, retVal); */
  return retVal;
}

const char *CCfg::get(const char *item, const char *def) const
{
  auto compareFunction = [item](const std::pair<std::string, std::string> &elem) {
    return elem.first.compare(std::string(item)) == 0;
  };

  if (item == nullptr)
  {
    return def;
  }

  auto it = std::find_if(configuration.begin(), configuration.end(), compareFunction);
  /* UBX_LOG(LCAT_DEBUG, "item=\"%s\" def=\"%s\" => \"%s\"", item ? item :
   * "NULL", def ? def : "NULL", it != configuration.end() ? it->second.c_str()
   * : "NULL"); */
  return it != configuration.end() ? it->second.c_str() : def;
}

void CCfg::readConfigLine(const std::string &line)
{
  std::smatch match;

  std::regex_match(
    line, match, std::regex("^[^#.]*?(\\w+).*?[=]{0,1}.*?(\\\"*\\/*\\b\\S+\\b\\\"*).*$"));

  if (match.size() == 3)
  {
    configuration.push_back(std::make_pair(match[1].str(), match[2].str()));
    // UBX_LOG(LCAT_DEBUG, "index=%d name=\"%s\" data=\"%s\"",
    // configuration.size(), configuration.back().first.c_str(),
    // configuration.back().second.c_str());
  }
}

void CCfg::print() const
{
  UBX_LOG(LCAT_DEBUG, "Configuration:\n");
  for (const auto &it : configuration)
  {
    UBX_LOG(LCAT_DEBUG, "Index: %s, Data: %s\n", it.first.c_str(), it.second.c_str());
  }
}
