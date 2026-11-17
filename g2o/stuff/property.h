// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_PROPERTY_H_
#define G2O_PROPERTY_H_

#include <g2o/prefix.h>
#include <g2o/stuff/g2o_stuff_api.h>
#include <g2o/stuff/string_tools.h>

#include <map>
#include <sstream>
#include <string>

G2O_START_NAMESPACE

class G2O_STUFF_API BaseProperty {
 public:
  BaseProperty(const std::string name_);
  virtual ~BaseProperty();
  const std::string& name() { return _name; }
  virtual std::string toString() const = 0;
  virtual bool fromString(const std::string& s) = 0;

 protected:
  std::string _name;
};

template <typename T>
class Property : public BaseProperty {
 public:
  typedef T ValueType;
  Property(const std::string& name_) : BaseProperty(name_) {}
  Property(const std::string& name_, const T& v)
      : BaseProperty(name_), _value(v) {}
  void setValue(const T& v) { _value = v; }
  const T& value() const { return _value; }
  virtual std::string toString() const {
    std::stringstream sstr;
    sstr << _value;
    return sstr.str();
  }
  virtual bool fromString(const std::string& s) {
    bool status = convertString(s, _value);
    return status;
  }

 protected:
  T _value;
};

/**
 * \brief a collection of properties mapping from name to the property itself
 */
class G2O_STUFF_API PropertyMap
    : protected std::map<std::string, BaseProperty*> {
 public:
  typedef std::map<std::string, BaseProperty*> BaseClass;
  typedef BaseClass::iterator PropertyMapIterator;
  typedef BaseClass::const_iterator PropertyMapConstIterator;

  PropertyMap();
  ~PropertyMap();

  /**
   * add a property to the map
   */
  bool addProperty(BaseProperty* p);

  /**
   * remove a property from the map
   */
  bool eraseProperty(const std::string& name_);

  /**
   * return a property by its name
   */
  template <typename P>
  P* getProperty(const std::string& name_) {
    auto it = find(name_);
    if (it == end()) return nullptr;
    return dynamic_cast<P*>(it->second);
  }
  template <typename P>
  const P* getProperty(const std::string& name_) const {
    auto it = find(name_);
    if (it == end()) return 0;
    return dynamic_cast<P*>(it->second);
  }

  /**
   * create a property and insert it
   */
  template <typename P>
  P* makeProperty(const std::string& name_, const typename P::ValueType& v) {
    auto it = find(name_);
    if (it == end()) {
      auto* p = new P(name_, v);
      addProperty(p);
      return p;
    } else
      return dynamic_cast<P*>(it->second);
  }

  /**
   * update a specfic property with a new value
   * @return true if the params is stored and update was carried out
   */
  bool updatePropertyFromString(const std::string& name,
                                const std::string& value);

  /**
   * update the map based on a name=value string, e.g., name1=val1,name2=val2
   * @return true, if it was possible to update all parameters
   */
  bool updateMapFromString(const std::string& values);

  void writeToCSV(std::ostream& os) const;

  using BaseClass::begin;
  using BaseClass::const_iterator;
  using BaseClass::end;
  using BaseClass::iterator;
  using BaseClass::size;
};

typedef Property<int> IntProperty;
typedef Property<bool> BoolProperty;
typedef Property<float> FloatProperty;
typedef Property<double> DoubleProperty;
typedef Property<std::string> StringProperty;

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
