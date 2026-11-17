// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/tictoc.h>

#include <g2o/stuff/timeutil.h>

#include <map>
#include <limits>
#include <algorithm>
#include <vector>
#include <cstdio>
#include <cstdlib>

G2O_START_NAMESPACE

  /**
   * \brief Internal structure of the tictoc profiling
   */
  struct TicTocElement
  {
    double ticTime;                         ///< the time of the last tic
    double totalTime;                       ///< the total time of this part of the algorithm
    int numCalls;                           ///< the number of calls
    double minTime;
    double maxTime;
    double exponentialMovingAverage;        ///< exponential moving average with alpha = 0.01
    std::string algorithmPart;              ///< name / description of the code block
    bool clockIsRunning;
    TicTocElement() :
      ticTime(0.), totalTime(0.), numCalls(0),
      minTime(std::numeric_limits<double>::max()),
      maxTime(0.), exponentialMovingAverage(0.),
      clockIsRunning(true)
    {}
    bool operator<(const TicTocElement& other) const
    {
      return totalTime < other.totalTime;
    }
  };
  typedef std::map<std::string, TicTocElement> TicTocMap;

  /**
   * \brief helper for printing the struct at the end of the lifetime of the program
   */
  struct TicTocInitializer
  {
    TicTocMap tictocElements;
    bool enabled;
    TicTocInitializer()
    {
      enabled = getenv("G2O_ENABLE_TICTOC") != nullptr;
    }
    ~TicTocInitializer()
    {
      if (!enabled) {
        return;
      }

      if (!tictocElements.empty()) {
        int longestName = 0;
        // sort the elements according to the total time and print a table
        std::vector<TicTocElement> sortedElements;
        sortedElements.reserve(tictocElements.size());
        for (TicTocMap::const_iterator it = tictocElements.begin(); it != tictocElements.end(); ++it) {
          if (it->second.numCalls == 0)
            continue;
          longestName = std::max(longestName, (int)it->first.size());
          sortedElements.push_back(it->second);
        }
        std::sort(sortedElements.begin(), sortedElements.end());

        longestName += 4;

        // now print the table to stdout
        printf("------------------------------------------\n");
        printf("|          TICTOC STATISTICS             |\n");
        printf("------------------------------------------\n");
        for(std::vector<TicTocElement>::const_iterator it = sortedElements.begin(); it != sortedElements.end(); ++it) {
          double avgTime = it->totalTime / it->numCalls;
          printf("%s", it->algorithmPart.c_str());
          for (int i = it->algorithmPart.size(); i < longestName; ++i)
            putchar(' ');
          printf("numCalls= %d\t total= %.4f\t avg= %.4f\t min= %.4f\t max= %.4f\t ema= %.4f\n",
              it->numCalls, it->totalTime, avgTime, it->minTime, it->maxTime, it->exponentialMovingAverage);
        }
        printf("------------------------------------------\n");
      }
    }
  };

  double tictoc(const char* algorithmPart)
  {
    static TicTocInitializer initializer;
    if (! initializer.enabled)
      return 0.;

    TicTocMap& tictocElements = initializer.tictocElements;
    static double alpha = 0.01;
    double now = get_monotonic_time();

    double dt = 0.;
    auto foundIt = tictocElements.find(algorithmPart);
    if (foundIt == tictocElements.end()) {
      // insert element
      TicTocElement e;
      e.ticTime = now;
      e.algorithmPart = algorithmPart;
      tictocElements[e.algorithmPart] = e;
    } else {
      if (foundIt->second.clockIsRunning) {
        dt = now - foundIt->second.ticTime;
        foundIt->second.totalTime += dt;
        foundIt->second.minTime = std::min(foundIt->second.minTime, dt);
        foundIt->second.maxTime = std::max(foundIt->second.maxTime, dt);
        if (foundIt->second.numCalls == 0)
          foundIt->second.exponentialMovingAverage = dt;
        else
          foundIt->second.exponentialMovingAverage = (1. - alpha) * foundIt->second.exponentialMovingAverage + alpha*dt;
        foundIt->second.numCalls++;
      } else {
        foundIt->second.ticTime = now;
      }
      foundIt->second.clockIsRunning = !foundIt->second.clockIsRunning;
    }
    return dt;
  }

  ScopedTictoc::ScopedTictoc(const char* algorithmPart) :
    _algorithmPart(algorithmPart)
  {
    tictoc(_algorithmPart.c_str());
  }

  ScopedTictoc::~ScopedTictoc()
  {
    tictoc(_algorithmPart.c_str());
  }

G2O_END_NAMESPACE
