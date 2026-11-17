// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2018 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_FACTORY_H
#define G2O_FACTORY_H

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <g2o/core/creators.h>
#include <g2o/stuff/misc.h>

// define to get some verbose output
//#define G2O_DEBUG_FACTORY

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  /**
   * \brief create vertices and edges based on TAGs in, for example, a file
   */
  class G2O_CORE_API Factory
  {
    public:

      //! return the instance
      static Factory* instance();

      //! free the instance
      G2O_CORE_DEPRECATED static void destroy();

      /**
       * register a tag for a specific creator
       */
      void registerType(const std::string& tag, AbstractHyperGraphElementCreator* c);

      /**
       * unregister a tag for a specific creator
       */
      void unregisterType(const std::string& tag);

      /**
       * construct a graph element based on its tag
       */
      HyperGraph::HyperGraphElement* construct(const std::string& tag) const;

      /**
       * construct a graph element based on its tag, but only if it's type (a bitmask) matches. A bitmask without any
       * bit set will construct any item. Otherwise a bit has to be set to allow construction of a graph element.
       */
      HyperGraph::HyperGraphElement* construct(const std::string& tag, const HyperGraph::GraphElemBitset& elemsToConstruct) const;

      /**
       * return whether the factory knows this tag or not
       */
      bool knowsTag(const std::string& tag, int* elementType = nullptr) const;

      //! return the TAG given a vertex
      const std::string& tag(const HyperGraph::HyperGraphElement* v) const;

      /**
       * get a list of all known types
       */
      void fillKnownTypes(std::vector<std::string>& types) const;

      /**
       * print a list of the known registered types to the given stream
       */
      void printRegisteredTypes(std::ostream& os, bool comment = false) const;

    protected:
      class CreatorInformation
      {
        public:
          std::unique_ptr<AbstractHyperGraphElementCreator> creator;
		  int elementTypeBit{ -1 };

          ~CreatorInformation()
          {
#ifdef G2O_DEBUG_FACTORY
            std::cout << "Deleting " << static_cast<void*>(creator.get()) << std::endl;
#endif
          }
      };

      typedef std::map<std::string, std::unique_ptr<CreatorInformation> >               CreatorMap;
      typedef std::map<std::string, std::string>                      TagLookup;
	  Factory() = default;
      ~Factory();

      CreatorMap _creator;     ///< look-up map for the existing creators
      TagLookup _tagLookup;    ///< reverse look-up, class name to tag

  private:
	  Factory(const Factory&) = delete;
	  Factory& operator=(const Factory&) = delete;
  };

  template<typename T>
  class RegisterTypeProxy
  {
    public:
      RegisterTypeProxy(const std::string& name)
          : _name(name)
      {
#ifdef G2O_DEBUG_FACTORY
        std::cout << __FUNCTION__ << ": Registering " << _name << " of type " << typeid(T).name() << std::endl;
#endif
        Factory::instance()->registerType(_name, new HyperGraphElementCreator<T>);
      }

      ~RegisterTypeProxy()
      {
#ifdef G2O_DEBUG_FACTORY
        std::cout << __FUNCTION__ << ": Unregistering " << _name << " of type " << typeid(T).name() << std::endl;
#endif
        Factory::instance()->unregisterType(_name);
      }

    private:
      std::string _name;
  };

  // These macros are used to automate registering types and forcing linkage
#define G2O_REGISTER_TYPE(name, classname) \
    extern "C" void G2O_FACTORY_EXPORT g2o_type_##classname(void) {} \
    static g2o::RegisterTypeProxy<classname> g_type_proxy_##classname(#name);

#define G2O_USE_TYPE_BY_CLASS_NAME(classname) \
    extern "C" void G2O_FACTORY_IMPORT g2o_type_##classname(void); \
    static g2o::ForceLinker proxy_##classname(g2o_type_##classname);

#define G2O_REGISTER_TYPE_GROUP(typeGroupName) \
    extern "C" void G2O_FACTORY_EXPORT g2o_type_group_##typeGroupName(void) {}

#define G2O_USE_TYPE_GROUP(typeGroupName) \
    extern "C" void G2O_FACTORY_IMPORT g2o_type_group_##typeGroupName(void); \
    static g2o::ForceLinker g2o_force_type_link_##typeGroupName(g2o_type_group_##typeGroupName);
G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
