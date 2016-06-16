// -*- C++ -*-
/*!
 * @file  GazeController.cpp
 * @brief gaze controller component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "GazeController.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include <hrpsys/util/Hrpsys.h>
#include <boost/assign.hpp>

#define MAX_TRANSITION_COUNT (static_cast<int>(2/m_dt))
typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* gazecontroller_spec[] =
    {
        "implementation_id", "GazeController",
        "type_name",         "GazeController",
        "description",       "gaze controller component",
        "version",           HRPSYS_PACKAGE_VERSION,
        "vendor",            "AIST",
        "category",          "example",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "10",
        "language",          "C++",
        "lang_type",         "compile",
        // Configuration variables
        "conf.default.debugLevel", "0",
        ""
    };
// </rtc-template>

GazeController::GazeController(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_qRefIn("qRef", m_qRef),
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_rpyIn("rpy", m_rpy),
      m_qOut("q", m_qRef),
      m_GazeControllerServicePort("GazeControllerService"),
      // </rtc-template>
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      dummy(0),
      use_sh_base_pos_rpy(false)
{
    m_service0.gaze(this);
}

GazeController::~GazeController()
{
}

RTC::ReturnCode_t GazeController::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("qRef", m_qRefIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("rpy", m_rpyIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);

    // Set service provider to Ports
    m_GazeControllerServicePort.registerProvider("service0", "GazeControllerService", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_GazeControllerServicePort);

    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable

    // </rtc-template>

    RTC::Properties& prop = getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());

    m_robot = hrp::BodyPtr(new hrp::Body());

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), 
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                 )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
    }
#if 0
    for ( int i = 0 ; i < dof; i++ ){
      if ( i != m_robot->joint(i)->jointId ) {
        std::cerr << "[" << m_profile.instance_name << "] jointId is not equal to the index number" << std::endl;
        return RTC::RTC_ERROR;
      }
    }
#endif

#if 0
    // setting from conf file
    // rleg,TARGET_LINK,BASE_LINK,x,y,z,rx,ry,rz,rth #<=pos + rot (axis+angle)
    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    std::map<std::string, std::string> base_name_map;
    if (end_effectors_str.size() > 0) {
        size_t prop_num = 10;
        size_t num = end_effectors_str.size()/prop_num;
        for (size_t i = 0; i < num; i++) {
            std::string ee_name, ee_target, ee_base;
            coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
            coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
            coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
            ee_trans eet;
            for (size_t j = 0; j < 3; j++) {
                coil::stringTo(eet.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
            }
            double tmpv[4];
            for (int j = 0; j < 4; j++ ) {
                coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
            }
            eet.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
            eet.target_name = ee_target;
            ee_map.insert(std::pair<std::string, ee_trans>(ee_name , eet));
            base_name_map.insert(std::pair<std::string, std::string>(ee_name, ee_base));
            std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << ee_target << " " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   target = " << ee_target << ", base = " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localPos = " << eet.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localR = " << eet.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        }
    }
#endif

    int num_cam = m_robot->numSensors(hrp::Sensor::VISION);
    for (int i = 0; i < num_cam; i++) {
        hrp::VisionSensor *s = m_robot->sensor<hrp::VisionSensor> (i);
        // s->link;
        // s->localR;
        // s->localPos;
        std::cerr << "v " << i << " / sensor: " << s->name << std::endl;
        if (i == 0) {
            tcam.name = s->name;
            tcam.link = s->link;
            tcam.localR = s->localR;
            tcam.localPos = s->localPos;
        }
    }


    unsigned int dof = m_robot->numJoints();
    // allocate memory for outPorts
    //m_q.data.length(dof);
    loop = 0;

    hv_org.resize(2);
    controllers.resize(2);
    TwoDofController::TwoDofControllerParam p;
    p.ke = 1;
    p.tc = 0.02;
    p.dt = 0.002;
    controllers[0].setup(p);
    controllers[0].reset();
    controllers[1].setup(p);
    controllers[1].reset();

    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    return RTC::RTC_OK;
}



RTC::ReturnCode_t GazeController::onFinalize()
{
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t GazeController::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t GazeController::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t GazeController::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t GazeController::onDeactivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
#if 0
    for ( std::map<std::string, GazeParam>::iterator it = m_gaze_param.begin(); it != m_gaze_param.end(); it++ ) {
        if (it->second.is_active) {
            stopGazeControllerNoWait(it->first);
            it->second.transition_count = 1;
        }
    }
#endif
    return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t GazeController::onExecute(RTC::UniqueId ec_id)
{
    std::cout << "GazeController::onExecute(" << ec_id << ")" << std::endl;
    loop ++;
    bool _publish = false;
    // check dataport input
    if (m_basePosIn.isNew()) {
        m_basePosIn.read();
    }
    //std::cout << "GazeController::onExecute(" << ec_id << ") a" << std::endl;
    if (m_baseRpyIn.isNew()) {
        m_baseRpyIn.read();
    }
    //std::cout << "GazeController::onExecute(" << ec_id << ") b" << std::endl;
    if (m_rpyIn.isNew()) {
        m_rpyIn.read();
    }
    //std::cout << "GazeController::onExecute(" << ec_id << ") c" << std::endl;
    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
    }
    //std::cout << "GazeController::onExecute(" << ec_id << ") d" << std::endl;
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
        _publish = true;
    }

    //std::cout << "GazeController::onExecute(" << ec_id << ") e" << std::endl;
    if ( m_qRef.data.length() ==  m_robot->numJoints() &&
         m_qCurrent.data.length() ==  m_robot->numJoints() ) {

        if ( DEBUGP ) {
          std::cerr << "[" << m_profile.instance_name << "] qRef = ";
            for ( int i = 0; i <  m_qRef.data.length(); i++ ){
                std::cerr << " " << m_qRef.data[i];
            }
            std::cerr << std::endl;
        }
#define PRINT_VEC3(name, arg) std::cerr << name << arg(0) << " " << arg(1) << " "  << arg(2)  << std::endl
        //std::cout << "GazeController::onExecute(" << ec_id << ") f" << std::endl;
        Guard guard(m_mutex);

        hrp::dvector q_org(m_robot->numJoints());
        std::vector<int> head_ids;
        // debug
        head_ids.resize(2);
        head_ids[0] = 15;
        head_ids[1] = 16;
        // reference model
        for ( int i = 0; i < m_robot->numJoints(); i++ ){
            q_org[i] = m_robot->joint(i)->q;
            m_robot->joint(i)->q = m_qRef.data[i];
        }
        for (int i = 0; i < head_ids.size(); i++) {
            m_robot->joint(head_ids[i])->q = hv_org[i];
        }
        PRINT_VEC3("root : ", m_robot->rootLink()->p);
        m_robot->rootLink()->p = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
        m_robot->rootLink()->R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
        m_robot->calcForwardKinematics();
        //std::cout << "GazeController::onExecute(" << ec_id << ") f1" << std::endl;

        //
        std::vector<double> head_vec(head_ids.size());
        for (int i = 0; i < head_ids.size(); i++) {
            head_vec[i] = m_robot->joint(head_ids[i])->q;
        }
        //std::cout << "GazeController::onExecute(" << ec_id << ") f2" << std::endl;
        hrp::Vector3 targetWorld;
        // debug
        targetWorld(0) = 4.0;
        targetWorld(1) = 0.0;
        targetWorld(2) = 0.0;
        //

        hrp::Vector3 tgt_org;
        {
            hrp::Matrix33 camR = tcam.link->R * tcam.localR;
            hrp::Vector3  camp = tcam.link->R * tcam.localPos + tcam.link->p;
            PRINT_VEC3("tcam_local: ", tcam.localPos);
            PRINT_VEC3("tcam_linkp: ", tcam.link->p);
            PRINT_VEC3("camp: ", camp);
            //std::cout << "GazeController::onExecute(" << ec_id << ") f3" << std::endl;
            tgt_org = camR.transpose() * (targetWorld - camp);
            //
            PRINT_VEC3("tgt: ", tgt_org);
        }
        //std::cout << "GazeController::onExecute(" << ec_id << ") g" << std::endl;
        // generate Jacobian
        hrp::dmatrix J(2, head_ids.size());
        for (int i = 0; i < head_ids.size(); i++) {
            // set joint angle
            for (int j = 0; j < head_ids.size(); j++) {
                if (i == j) {
                    m_robot->joint(head_ids[j])->q = head_vec[j] + 0.001; // magic number
                } else {
                    m_robot->joint(head_ids[j])->q = head_vec[j];
                }
            }
            m_robot->calcForwardKinematics();
            hrp::Matrix33 camR = tcam.link->R * tcam.localR;
            hrp::Vector3  camp = tcam.link->R * tcam.localPos + tcam.link->p;

            hrp::Vector3 diff = (camR.transpose() * (targetWorld - camp)) - tgt_org;
            J(0, i) = diff(0);
            J(1, i) = diff(1);
        }
        J *= (1 / 0.001);
        std::cerr << "J" << std::endl;
        std::cerr << "   " << J(0 ,0) << " " << J(0, 1) << std::endl;
        std::cerr << "   " << J(1 ,0) << " " << J(1, 1) << std::endl;
        hrp::dmatrix Jt(head_ids.size(), 2);
        int ret = hrp::calcPseudoInverse(J, Jt);
        std::cerr << "Jt" << std::endl;
        std::cerr << "   " << Jt(0 ,0) << " " << Jt(0, 1) << std::endl;
        std::cerr << "   " << Jt(1 ,0) << " " << Jt(1, 1) << std::endl;
        hrp::dvector ret_q(head_ids.size());
        hrp::dvector tgt(2);
        tgt(0) = - tgt_org[0];
        tgt(1) = - tgt_org[1];
        ret_q = Jt * tgt;
        //hv_org[0] = hv_org[0] + ret_q(0);
        //hv_org[1] = hv_org[1] + ret_q(1);
        hv_org[0] = controllers[0].update(hv_org[0], hv_org[0] + ret_q(0));
        hv_org[1] = controllers[1].update(hv_org[1], hv_org[1] + ret_q(1));
        std::cerr << "retq: " << ret_q(0) << " " << ret_q(1) << std::endl;
        std::cerr << "hv_org: " << hv_org[0] << " " << hv_org[1] << std::endl;
        std::cerr << std::endl;

        //speed limit

        for (int i = 0; i < head_ids.size(); i++) {
            m_qRef.data[head_ids[i]] = hv_org[i];
        }
        //std::cout << "GazeController::onExecute(" << ec_id << ") h" << std::endl;
    } else {
        if ( DEBUGP || loop % 100 == 0 ) {
            std::cerr << "GazeController is not working..." << std::endl;
            std::cerr << "         m_qRef " << m_qRef.data.length() << std::endl;
            std::cerr << "     m_qCurrent " << m_qCurrent.data.length() << std::endl;
        }
    }
    if(_publish) {
        m_qOut.write();
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t GazeController::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t GazeController::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t GazeController::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t GazeController::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t GazeController::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

extern "C"
{

    void GazeControllerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(gazecontroller_spec);
        manager->registerFactory(profile,
                                 RTC::Create<GazeController>,
                                 RTC::Delete<GazeController>);
    }

};
