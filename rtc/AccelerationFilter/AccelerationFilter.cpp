// -*- C++ -*-
/*!
 * @file  AccelerationFilter.cpp * @brief Acceleration Filter component * $Date$ 
 *
 * $Id$ 
 */
#include "AccelerationFilter.h"

// Module specification
// <rtc-template block="module_spec">
static const char* accelerationfilter_spec[] =
  {
    "implementation_id", "AccelerationFilter",
    "type_name",         "AccelerationFilter",
    "description",       "Acceleration Filter component",
    "version",           "1.0",
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.use_filter", "false",
    ""
  };
// </rtc-template>

AccelerationFilter::AccelerationFilter(RTC::Manager* manager)
    // <rtc-template block="initializer">
    : RTC::DataFlowComponentBase(manager),
      m_accInIn("accIn", m_accIn),
      m_rateInIn("rateIn", m_rateIn),
      m_rpyInIn("rpyIn", m_rpyIn),
      m_posInIn("posIn", m_posIn),
      m_velOutOut("velOut", m_velOut),
      m_posOutOut("posOut", m_posOut),
      m_AccelerationFilterServicePort("AccelerationFilterService"),
    // </rtc-template>
      m_use_filter_bool(false)
{
}

AccelerationFilter::~AccelerationFilter()
{
}


RTC::ReturnCode_t AccelerationFilter::onInitialize()
{
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("accIn", m_accInIn);
    addInPort("rateIn", m_rateInIn);
    addInPort("rpyIn", m_rpyInIn);
    addInPort("posIn", m_posInIn);

    // Set OutPort buffer
    addOutPort("velOut", m_velOutOut);
    addOutPort("posOut", m_posOutOut);

    // Set service provider to Ports
    m_AccelerationFilterServicePort.registerProvider("service0", "AccelerationFilterService", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_AccelerationFilterServicePort);

    // </rtc-template>

    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("use_filter", m_use_filter, "false");
    if (m_use_filter == "true" ) {
        m_use_filter_bool = true;
    }
    RTC::Properties& prop = getProperties();
    if ( ! coil::stringTo(m_dt, prop["dt"].c_str()) ) {
        std::cerr << "[" << m_profile.instance_name << "]failed to get dt" << std::endl;
        return RTC::RTC_ERROR;
    }
    // read gravity param
    // read filter param
    // read reset threshold -> min_vel

    // </rtc-template>
    return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t AccelerationFilter::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t AccelerationFilter::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}
RTC::ReturnCode_t AccelerationFilter::onDeactivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    // reset filter
    return RTC::RTC_OK;
}


RTC::ReturnCode_t AccelerationFilter::onExecute(RTC::UniqueId ec_id)
{
    if (m_rpyInIn.isNew()) {
        m_rpyInIn.read();
    }
    if (m_rateInIn.isNew()) {
        m_rateInIn.read();
    }
    if (m_posInIn.isNew()) {
        m_posInIn.read();
    }
    //
    if (m_accInIn.isNew()) {
        m_accInIn.read();

        hrp::Vector3 gravity(0, 0, -9.8);
        hrp::Vector3 acc(m_accIn.data.ax, m_accIn.data.ay, m_accIn.data.az);
        hrp::Matrix33 imuR = hrp::rotFromRpy(m_rpyIn.data.r,
                                             m_rpyIn.data.p,
                                             m_rpyIn.data.y);
        hrp::Vector3 acc_wo_g = imuR * acc + gravity;

        for (int i = 0; i < 3; i++) {
#if 0
            if (std::abs(_estimated_vel[i]) < min_vel) {
                global_velocity[i] = 0;
                prev_raw_acc[i] = 0;
                prev_filtered_acc[i] = 0;
                continue;
            }
#endif
            if (m_use_filter_bool) {
                double filtered_acc =  m_filters[i].passFilter(acc_wo_g[i]);
                m_global_vel[i] += filtered_acc * m_dt;
            } else {
                m_global_vel[i] += acc_wo_g[i] * m_dt;
            }
        }
        hrp::Vector3 _result_vel = imuR.inverse() * m_global_vel; // result should be described in sensor coords
    }

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AccelerationFilter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
  void AccelerationFilterInit(RTC::Manager* manager)
  {
    coil::Properties profile(accelerationfilter_spec);
    manager->registerFactory(profile,
                             RTC::Create<AccelerationFilter>,
                             RTC::Delete<AccelerationFilter>);
  }
};



