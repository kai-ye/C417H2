//#line 2 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"
// *********************************************************
// 
// File autogenerated for the aqua_utils package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

#ifndef __aqua_utils__GPSEMULATORCONFIG_H__
#define __aqua_utils__GPSEMULATORCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace aqua_utils
{
  class GPSEmulatorConfigStatics;
  
  class GPSEmulatorConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(GPSEmulatorConfig &config, const GPSEmulatorConfig &max, const GPSEmulatorConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const GPSEmulatorConfig &config1, const GPSEmulatorConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, GPSEmulatorConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const GPSEmulatorConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, GPSEmulatorConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const GPSEmulatorConfig &config) const = 0;
      virtual void getValue(const GPSEmulatorConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T GPSEmulatorConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (GPSEmulatorConfig::* field);

      virtual void clamp(GPSEmulatorConfig &config, const GPSEmulatorConfig &max, const GPSEmulatorConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const GPSEmulatorConfig &config1, const GPSEmulatorConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, GPSEmulatorConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const GPSEmulatorConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, GPSEmulatorConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const GPSEmulatorConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const GPSEmulatorConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, GPSEmulatorConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    template<class T, class PT>
    class GroupDescription : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string name, std::string type, int parent, int id, bool s, T PT::* f) : AbstractGroupDescription(name, type, parent, id, s), field(f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, GPSEmulatorConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<GPSEmulatorConfig::AbstractGroupDescriptionConstPtr> groups;
    };
    
class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(GPSEmulatorConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("pub_rate"==(*_i)->name){pub_rate = boost::any_cast<double>(val);}
        if("pub_probability"==(*_i)->name){pub_probability = boost::any_cast<double>(val);}
        if("target_latitude"==(*_i)->name){target_latitude = boost::any_cast<double>(val);}
        if("target_longitude"==(*_i)->name){target_longitude = boost::any_cast<double>(val);}
        if("stdev_latitude_m"==(*_i)->name){stdev_latitude_m = boost::any_cast<double>(val);}
        if("stdev_longitude_m"==(*_i)->name){stdev_longitude_m = boost::any_cast<double>(val);}
        if("speed_mps"==(*_i)->name){speed_mps = boost::any_cast<double>(val);}
      }
    }

    double pub_rate;
double pub_probability;
double target_latitude;
double target_longitude;
double stdev_latitude_m;
double stdev_longitude_m;
double speed_mps;

    bool state;
    std::string name;

    
}groups;



//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double pub_rate;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double pub_probability;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double target_latitude;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double target_longitude;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double stdev_latitude_m;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double stdev_longitude_m;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double speed_mps;
//#line 218 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("GPSEmulatorConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const GPSEmulatorConfig &__max__ = __getMax__();
      const GPSEmulatorConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const GPSEmulatorConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const GPSEmulatorConfig &__getDefault__();
    static const GPSEmulatorConfig &__getMax__();
    static const GPSEmulatorConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();
    
  private:
    static const GPSEmulatorConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void GPSEmulatorConfig::ParamDescription<std::string>::clamp(GPSEmulatorConfig &config, const GPSEmulatorConfig &max, const GPSEmulatorConfig &min) const
  {
    return;
  }

  class GPSEmulatorConfigStatics
  {
    friend class GPSEmulatorConfig;
    
    GPSEmulatorConfigStatics()
    {
GPSEmulatorConfig::GroupDescription<GPSEmulatorConfig::DEFAULT, GPSEmulatorConfig> Default("Default", "", 0, 0, true, &GPSEmulatorConfig::groups);
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.pub_rate = 0.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.pub_rate = 10.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.pub_rate = 1.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("pub_rate", "double", 0, "GPS message publish rate", "", &GPSEmulatorConfig::pub_rate)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("pub_rate", "double", 0, "GPS message publish rate", "", &GPSEmulatorConfig::pub_rate)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.pub_probability = 0.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.pub_probability = 1.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.pub_probability = 0.8;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("pub_probability", "double", 0, "Probability of publishing next GPS message", "", &GPSEmulatorConfig::pub_probability)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("pub_probability", "double", 0, "Probability of publishing next GPS message", "", &GPSEmulatorConfig::pub_probability)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.target_latitude = -90.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.target_latitude = 90.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.target_latitude = 13.1931;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("target_latitude", "double", 0, "Target latitude (deg)", "", &GPSEmulatorConfig::target_latitude)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("target_latitude", "double", 0, "Target latitude (deg)", "", &GPSEmulatorConfig::target_latitude)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.target_longitude = -180.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.target_longitude = 180.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.target_longitude = -59.6415;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("target_longitude", "double", 0, "Target longitude (deg)", "", &GPSEmulatorConfig::target_longitude)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("target_longitude", "double", 0, "Target longitude (deg)", "", &GPSEmulatorConfig::target_longitude)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.stdev_latitude_m = 0.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.stdev_latitude_m = 20.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.stdev_latitude_m = 2.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("stdev_latitude_m", "double", 0, "Gaussian standard deviation for latitude (pseudo-m converting to deg)", "", &GPSEmulatorConfig::stdev_latitude_m)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("stdev_latitude_m", "double", 0, "Gaussian standard deviation for latitude (pseudo-m converting to deg)", "", &GPSEmulatorConfig::stdev_latitude_m)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.stdev_longitude_m = 0.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.stdev_longitude_m = 20.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.stdev_longitude_m = 2.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("stdev_longitude_m", "double", 0, "Gaussian standard deviation for longitude (pseudo-m converting to deg)", "", &GPSEmulatorConfig::stdev_longitude_m)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("stdev_longitude_m", "double", 0, "Gaussian standard deviation for longitude (pseudo-m converting to deg)", "", &GPSEmulatorConfig::stdev_longitude_m)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.speed_mps = 0.01;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.speed_mps = 1000000.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.speed_mps = 1.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("speed_mps", "double", 0, "Emulated movement speed (pseudo-m/s)", "", &GPSEmulatorConfig::speed_mps)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(GPSEmulatorConfig::AbstractParamDescriptionConstPtr(new GPSEmulatorConfig::ParamDescription<double>("speed_mps", "double", 0, "Emulated movement speed (pseudo-m/s)", "", &GPSEmulatorConfig::speed_mps)));
//#line 233 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.convertParams();
//#line 233 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __group_descriptions__.push_back(GPSEmulatorConfig::AbstractGroupDescriptionConstPtr(new GPSEmulatorConfig::GroupDescription<GPSEmulatorConfig::DEFAULT, GPSEmulatorConfig>(Default)));
//#line 353 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"

      for (std::vector<GPSEmulatorConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__); 
    }
    std::vector<GPSEmulatorConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<GPSEmulatorConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    GPSEmulatorConfig __max__;
    GPSEmulatorConfig __min__;
    GPSEmulatorConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const GPSEmulatorConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static GPSEmulatorConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &GPSEmulatorConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const GPSEmulatorConfig &GPSEmulatorConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const GPSEmulatorConfig &GPSEmulatorConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const GPSEmulatorConfig &GPSEmulatorConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<GPSEmulatorConfig::AbstractParamDescriptionConstPtr> &GPSEmulatorConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<GPSEmulatorConfig::AbstractGroupDescriptionConstPtr> &GPSEmulatorConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const GPSEmulatorConfigStatics *GPSEmulatorConfig::__get_statics__()
  {
    const static GPSEmulatorConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = GPSEmulatorConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __GPSEMULATORRECONFIGURATOR_H__
