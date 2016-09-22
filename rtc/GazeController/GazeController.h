// -*- C++ -*-
/*!
 * @file  GazeController.h
 * @brief gaze control component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef GAZE_H
#define GAZE_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "GazeControllerService_impl.h"
#include "../Stabilizer/TwoDofController.h"
// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class GazeController
  : public RTC::DataFlowComponentBase
{
 public:
  GazeController(RTC::Manager* manager);
  virtual ~GazeController();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
  virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);

  bool startGazeController();
  bool stopGazeController(double tm);
  bool setGazeControllerParam(const OpenHRP::GazeControllerService::gazeParam &i_param_);
  bool getGazeControllerParam(OpenHRP::GazeControllerService::gazeParam &i_param_);
  bool waitGazeControllerTransition();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qCurrent;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;
  TimedOrientation3D m_rpy;
  InPort<TimedOrientation3D> m_rpyIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_q;
  OutPort<TimedDoubleSeq> m_qOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_GazeControllerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  GazeControllerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>

 private:
  struct camera_trans {
    std::string   name;
    hrp::Vector3  localPos;
    hrp::Matrix33 localR;
    hrp::Link     *link;
  };

  void gazeControll (void);
  void transition (void);
  bool setTargetPos(hrp::Vector3 &worldpos);

  double m_dt;
  unsigned int m_debugLevel;

  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;

  std::vector<double > gaze_angles;
  std::vector<int >    gaze_joints;
  struct camera_trans tcam;
  std::vector<class TwoDofController> controllers;

  hrp::Link *target_link_;
  std::string target_name_;
  hrp::Vector3 target_pos_;
  //hrp::Vector3 target_rot;

  OpenHRP::GazeControllerService::BaseType base_type_;
  bool use_estimated_rpy_;
  bool initialized_;
  int loop_;
  int transition_count_;
  int max_transition_count_;
};

extern "C"
{
  void GazeControllerInit(RTC::Manager* manager);
};

#endif // GAZE_H
