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
    "conf.default.high_pass_filter_freq", "0.1",
    "conf.default.low_pass_filter_freq", "50",
    "conf.default.use_low_pass_filter", "false",
    ""
  };
// </rtc-template>

AccelerationFilter::AccelerationFilter(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_accInIn("accIn", m_accIn),
    m_rpyInIn("rpyIn", m_rpyIn),
    m_rateInIn("rateIn", m_rateIn),
    m_velOutOut("velOut", m_velOut),
    m_posOutOut("posOut", m_posOut),
    m_AccelerationFilterServicePort("AccelerationFilterService")

    // </rtc-template>
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
  addInPort("rpyIn", m_rpyInIn);
  addInPort("rateIn", m_rateInIn);

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
  bindParameter("high_pass_filter_freq", m_high_pass_filter_freq, "0.1");
  bindParameter("low_pass_filter_freq", m_low_pass_filter_freq, "50");
  bindParameter("use_low_pass_filter", m_use_low_pass_filter, "false");

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
/*
RTC::ReturnCode_t AccelerationFilter::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
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



