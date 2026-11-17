// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TICTOC_H
#define G2O_TICTOC_H

#include <g2o/core/abi.h>
#include <g2o/stuff/g2o_stuff_api.h>

#include <string>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  /**
   * \brief Profile the timing of certain parts of your algorithm.
   *
   * Profile the timing of certain parts of your algorithm.
   * A typical use-case is as follows:
   *
   * tictoc("doSomething");
   * // place the code here.
   * tictoc("doSomething");
   *
   * This will calculate statistics for the operations within
   * the two calls to tictoc()
   *
   * If the environment variable G2O_ENABLE_TICTOC is defined, the timing will
   * be performed.
   */
   G2O_STUFF_API double tictoc(const char* algorithmPart);

   /**
    * \brief Simplify calls to tictoc() for a whole scope
    *
    * See also the macro G2O_SCOPED_TICTOC below.
    */
   class G2O_STUFF_API ScopedTictoc
   {
     public:
       ScopedTictoc(const char* algorithmPart);
       ~ScopedTictoc();
     protected:
       std::string _algorithmPart;
   };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#ifndef G2O_SCOPED_TICTOC
#define G2O_SCOPED_TICTOC(s) \
  g2o::ScopedTictoc scopedTictoc (s)
#endif

#endif
