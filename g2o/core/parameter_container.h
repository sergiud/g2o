// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_GRAPH_PARAMETER_CONTAINER_HH_
#define G2O_GRAPH_PARAMETER_CONTAINER_HH_

#include <iosfwd>
#include <map>
#include <string>

#include <g2o/core/abi.h>
#include <g2o/core/g2o_core_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

    class Parameter;

    /**
     * \brief map id to parameters
     */
    class G2O_CORE_API ParameterContainer : protected std::map<int, Parameter*>
    {
    public:
      typedef std::map<int, Parameter*> BaseClass;

      /**
       * create a container for the parameters.
       * @param isMainStorage_ pointers to the parameters are owned by this container, i.e., freed in its constructor
       */
      ParameterContainer(bool isMainStorage_=true);
      virtual ~ParameterContainer();
      //! add parameter to the container
      bool addParameter(Parameter* p);
      //! return a parameter based on its ID
      Parameter* getParameter(int id);
      //! return a parameter based on its ID
      const Parameter* getParameter(int id) const;
      //! remove a parameter from the container, i.e., the user now owns the pointer
      Parameter* detachParameter(int id);
      //! read parameters from a stream
      virtual bool read(std::istream& is, const std::map<std::string, std::string>* renamedMap =nullptr);
      //! write the data to a stream
      virtual bool write(std::ostream& os) const;
      bool isMainStorage() const {return _isMainStorage;}
      void clear();

      // stuff of the base class that should re-appear
      using BaseClass::size;

    protected:
      bool _isMainStorage;
    };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
