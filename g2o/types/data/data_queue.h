// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_DATA_QUEUE_H
#define G2O_DATA_QUEUE_H

#include <map>

#include <g2o/core/abi.h>
#include <g2o/types/data/g2o_types_data_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  class RobotData;

  /**
   * \brief a simple queue to store data and retrieve based on a timestamp
   */
  class G2O_TYPES_DATA_API DataQueue
  {
    public:
      typedef std::map<double, RobotData*>           Buffer;

    public:
      DataQueue();
      ~DataQueue();

      void add(RobotData* rd);

      RobotData* findClosestData(double timestamp) const;

      RobotData* before(double timestamp) const;
      RobotData* after(double timestamp) const;

      const Buffer& buffer() const {return _buffer;}

    protected:
      Buffer _buffer;
  };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
