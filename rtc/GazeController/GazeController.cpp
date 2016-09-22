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
// for atoi
#include <cstdlib>


#define TRANSITION_TIME 2.0
#define MAX_TRANSITION_COUNT (static_cast<int>(TRANSITION_TIME/m_dt))
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
      m_debugLevel(0),
      m_robot(hrp::BodyPtr()),
      base_type_(OpenHRP::GazeControllerService::BAD_BASE_TYPE),
      use_sh_base_pos_rpy_(false),
      initialized_(false),
      transition_count_(0)
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

    unsigned int dof = m_robot->numJoints();

    //coil::vstring camera_str;
    //coil::stringTo(camname, camera_str);
    std::string camname = prop["gaze_camera"];

    int num_cam = m_robot->numSensors(hrp::Sensor::VISION);
    if (num_cam == 0) {
        std::cerr << "[" << m_profile.instance_name << "] There is no CAMERA model."<< std::endl;
        return RTC::RTC_ERROR;
    }
    tcam.link = NULL;
    for (int i = 0; i < num_cam; i++) {
        hrp::VisionSensor *s = m_robot->sensor<hrp::VisionSensor> (i);
        //std::cerr << "v " << i << " / sensor: " << s->name << std::endl;
        if (s->name == camname) {
            std::cerr << "[" << m_profile.instance_name << "] CAMERA (" << camname << ") is used for control."<< std::endl;
            tcam.name = s->name;
            tcam.link = s->link;
            tcam.localR = s->localR;
            tcam.localPos = s->localPos;
        }
    }
    if (tcam.link == NULL) {
        hrp::VisionSensor *s = m_robot->sensor<hrp::VisionSensor> (0);
        tcam.name = s->name;
        tcam.link = s->link;
        tcam.localR = s->localR;
        tcam.localPos = s->localPos;
        std::cerr << "[" << m_profile.instance_name << "] CAMERA (" << camname << ") was not found."<< std::endl;
        std::cerr << "[" << m_profile.instance_name << "] CAMERA (" << tcam.name << ") is used for control."<< std::endl;
    }

    // setting from conf file
    coil::vstring joints_str = coil::split(prop["joints_for_gaze"], ",");
    if (joints_str.size() > 0) {
        for (size_t i = 0; i < joints_str.size(); i++) {
            std::string jointname;
            coil::stringTo(jointname, joints_str[i].c_str());
            int id = std::atoi(jointname.c_str());
            if (id == 0 && jointname[0] != '0') {
                // jointname is not a number
                hrp::Link* l = m_robot->link(jointname);
                if (!!l) {
                    int id = 0;
                    for(int si = 0; si <  m_robot->numJoints(); si++) {
                        if (m_robot->joint(si) == l) {
                            id = si;
                            break;
                        }
                    }
                    std::cerr << "[" << m_profile.instance_name << "] Joint (" << id << ") is used as index " << i;
                    std::cerr << ", set by \'" << jointname << "\'" << std::endl;
                    gaze_joints.push_back(id);
                }
            } else {
                // jointname is a number.
                if(id < dof) {
                    std::cerr << "[" << m_profile.instance_name << "] Joint (" << id << ") is used as index " << i;
                    std::cerr << ", set by \'" << jointname << "\'" << std::endl;
                    gaze_joints.push_back(id);
                }
            }
        }
    }
    //
    if (gaze_joints.size() < 1) {
        std::cerr << "[" << m_profile.instance_name << "] There is not enough joints (" << gaze_joints.size() << ")." << std::endl;
        return RTC::RTC_ERROR;
    }
    gaze_angles.resize(gaze_joints.size());

    TwoDofController::TwoDofControllerParam p;
    p.ke = 1;
    p.tc = 0.02;
    p.dt = m_dt;

    coil::vstring parameters_str = coil::split(prop["gaze_parameters"], ",");
    if (parameters_str.size() == 2) {
        double ke, tc;
        coil::stringTo(ke, parameters_str[0].c_str());
        coil::stringTo(tc, parameters_str[1].c_str());
        p.ke = ke;
        p.tc = tc;
    }
    controllers.resize(gaze_joints.size());
    for(int i = 0; i < gaze_joints.size(); i++) {
        controllers[i].setup(p);
        controllers[i].reset();
    }

    std::cerr << "[" << m_profile.instance_name << "] use TwoDofControllerParam";
    std::cerr << ", ke = " << p.ke << ", tc = " << p.tc << std::endl;

    loop_ = 0;
    initialized_ = true;
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

    transition_count_ = 0;
    loop_ = 0;

    return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop_%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t GazeController::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "GazeController::onExecute(" << ec_id << ")" << std::endl;
    loop_++;
    bool _publish = false;
    // check dataport input
    if (m_basePosIn.isNew()) {
        m_basePosIn.read();
    }
    if (m_baseRpyIn.isNew()) {
        m_baseRpyIn.read();
    }
    if (m_rpyIn.isNew()) {
        m_rpyIn.read();
    }
    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
    }
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
        _publish = true;
    }

    if ( initialized_ &&
         m_qRef.data.length() ==  m_robot->numJoints() &&
         m_qCurrent.data.length() ==  m_robot->numJoints() ) {

        if ( DEBUGP ) {
            std::cerr << "[" << m_profile.instance_name << "] qRef = ";
            for ( int i = 0; i <  m_qRef.data.length(); i++ ){
                std::cerr << " " << m_qRef.data[i];
            }
            std::cerr << std::endl;
        }

        if (transition_count_ == -1) {
            if ( DEBUGP || loop_ % 500 == 0 ) {
                std::cerr << "GazeController is working." << std::endl;
            }
            gazeControll();
            //stabilize();
        } else if (transition_count_ != 0) {
            transition();
        } else {
            // if(tansition_count == 0) pass through
        }
    } else {
        if ( DEBUGP || loop_ % 100 == 0 ) {
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

#define PRINT_VEC3(name, arg) std::cerr << name << arg(0) << " " << arg(1) << " "  << arg(2)  << std::endl;
void GazeController::gazeControll (void) {
    Guard guard(m_mutex);

    // reference model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_robot->joint(i)->q = m_qRef.data[i];
    }
    for (int i = 0; i < gaze_joints.size(); i++) {
        m_robot->joint(gaze_joints[i])->q = gaze_angles[i];
    }
    m_robot->rootLink()->p = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
    m_robot->rootLink()->R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    m_robot->calcForwardKinematics();

    std::vector<double> head_vec(gaze_joints.size());
    for (int i = 0; i < gaze_joints.size(); i++) {
        head_vec[i] = m_robot->joint(gaze_joints[i])->q;
    }

    hrp::Vector3 targetWorld;
    if (!setTargetPos(targetWorld)) {
        return;
    }

    hrp::Vector3 tgt_org;
    {
        hrp::Matrix33 camR = tcam.link->R * tcam.localR;
        hrp::Vector3  camp = tcam.link->R * tcam.localPos + tcam.link->p;
        //PRINT_VEC3("tcam_local: ", tcam.localPos);
        //PRINT_VEC3("tcam_linkp: ", tcam.link->p);
        //PRINT_VEC3("camp: ", camp);
        tgt_org = camR.transpose() * (targetWorld - camp);
        //PRINT_VEC3("tgt: ", tgt_org);
    }

#define DIFF_FOR_JACOBI 0.001
    // generate Jacobian
    hrp::dmatrix J(2, gaze_joints.size());
    for (int i = 0; i < gaze_joints.size(); i++) {
        // set joint angle
        for (int j = 0; j < gaze_joints.size(); j++) {
            if (i == j) {
                m_robot->joint(gaze_joints[j])->q = head_vec[j] + DIFF_FOR_JACOBI; // magic number
            } else {
                m_robot->joint(gaze_joints[j])->q = head_vec[j];
            }
        }
        m_robot->calcForwardKinematics();
        hrp::Matrix33 camR = tcam.link->R * tcam.localR;
        hrp::Vector3  camp = tcam.link->R * tcam.localPos + tcam.link->p;

        hrp::Vector3 diff = (camR.transpose() * (targetWorld - camp)) - tgt_org;
        J(0, i) = diff(0);
        J(1, i) = diff(1);
    }
    J *= (1 / DIFF_FOR_JACOBI);
#if 0
    std::cerr << "J" << std::endl;
    std::cerr << "   " << J(0 ,0) << " " << J(0, 1) << std::endl;
    std::cerr << "   " << J(1 ,0) << " " << J(1, 1) << std::endl;
#endif
    hrp::dmatrix Jt(gaze_joints.size(), 2);
    int ret = hrp::calcPseudoInverse(J, Jt);
#if 0
    std::cerr << "Jt" << std::endl;
    std::cerr << "   " << Jt(0 ,0) << " " << Jt(0, 1) << std::endl;
    std::cerr << "   " << Jt(1 ,0) << " " << Jt(1, 1) << std::endl;
#endif
    hrp::dvector ret_q(gaze_joints.size());
    hrp::dvector tgt(2);
    tgt(0) = - tgt_org[0];
    tgt(1) = - tgt_org[1];
    ret_q = Jt * tgt;

    for(int i = 0; i < gaze_angles.size(); i++) {
        gaze_angles[i] = controllers[i].update(gaze_angles[i], gaze_angles[i] + ret_q(i));
    }

    std::cerr << "retq:";
    for(int i = 0; i < gaze_angles.size(); i++) {
        std::cerr << " " << ret_q(i);
    }
    std::cerr << std::endl;

    for(int i = 0; i < gaze_angles.size(); i++) {
        std::cerr << " " << gaze_angles[i];
    }
    std::cerr << std::endl;


    for (int i = 0; i < gaze_joints.size(); i++) {
        m_qRef.data[gaze_joints[i]] = gaze_angles[i];
    }
}
void GazeController::transition (void) {
    std::vector<double > ref_vec(gaze_angles.size());
    transition_count_--;
    // reference model
    for (int i = 0; i < gaze_joints.size(); i++) {
        ref_vec[i] = m_qRef.data[gaze_joints[i]];
        gaze_angles[i] = gaze_angles[i] +  (MAX_TRANSITION_COUNT - transition_count_) * ((ref_vec[i] - gaze_angles[i])/MAX_TRANSITION_COUNT);
    }

    for (int i = 0; i < gaze_joints.size(); i++) {
        m_qRef.data[gaze_joints[i]] = gaze_angles[i];
    }
}

bool GazeController::setTargetPos(hrp::Vector3 &worldpos) {
    if (base_type_ == OpenHRP::GazeControllerService::WORLD_POS) {
        worldpos = target_pos_;
    } else if (base_type_ == OpenHRP::GazeControllerService::ROOT_POS &&
               base_type_ == OpenHRP::GazeControllerService::LINK_POS ) {
        worldpos = (target_link_->R * target_pos_) + target_link_->p;
    } else {
        return false;
    }

    return true;
}

bool GazeController::startGazeController() {
    if(transition_count_ == 0) {
        transition_count_ == -1; // start Gaze
        return true;
    }
    return false;
}

bool GazeController::stopGazeController() {
    if(transition_count_ == -1) {
        transition_count_ = MAX_TRANSITION_COUNT;
        return true;
    }
    return false;
}

bool GazeController::setGazeControllerParam(OpenHRP::GazeControllerService::gazeParam i_param_) {
    if(!initialized_) return false;

    {
        Guard guard(m_mutex);

        base_type_ = i_param_.base_type;
        use_sh_base_pos_rpy_ = i_param_.use_sh_base_pos_rpy;
        if(i_param_.base_type == OpenHRP::GazeControllerService::WORLD_POS) {
            target_link_ = NULL;
            target_name_ = "world";
        } else if(i_param_.base_type == OpenHRP::GazeControllerService::ROOT_POS) {
            target_link_ = m_robot->rootLink();
            target_name_ = "root";
        } else if(i_param_.base_type == OpenHRP::GazeControllerService::LINK_POS) {
            std::string linkname(i_param_.linkname);
            hrp::Link* l = m_robot->link(linkname);
            if(!link) {
                std::cerr << "[" << m_profile.instance_name << "] link(" << linkname << ") was not found." << std::endl;
                base_type_ = OpenHRP::GazeControllerService::BAD_BASE_TYPE;
                return false;
            }
            target_link_ = l;
            target_name_ = linkname;
        } else if(i_param_.base_type == OpenHRP::GazeControllerService::WORLD_ROT &&
                  i_param_.base_type == OpenHRP::GazeControllerService::ROOT_ROT  &&
                  i_param_.base_type == OpenHRP::GazeControllerService::LINK_ROT  &&
                  i_param_.base_type == OpenHRP::GazeControllerService::CURRENT_CAMERA_ROT) {
            std::cerr << "[" << m_profile.instance_name << "] base_type have not been implemented yet." << std::endl;
            base_type_ = OpenHRP::GazeControllerService::BAD_BASE_TYPE;
            return false;
        } else {
            std::cerr << "[" << m_profile.instance_name << "] BAD base_type" << std::endl;
            base_type_ = OpenHRP::GazeControllerService::BAD_BASE_TYPE;
            return false;
        }
        target_pos_(0) = i_param_.target[0];
        target_pos_(1) = i_param_.target[1];
        target_pos_(2) = i_param_.target[2];
        std::cerr << "[" << m_profile.instance_name << "] " << std::endl;
        if(!!target_link_) {
            std::cerr << "   target_link: " << target_link_->name;
        } else {
            std::cerr << "   target_name: " << target_name_;
        }
        std::cerr << "   target_pos: " << target_pos_(0) << " " << target_pos_(1) << " " << target_pos_(2) << std::endl;
        std::cerr << "   use_sh_rpy: " << use_sh_base_pos_rpy_;
    }
    return true;
}

bool GazeController::getGazeControllerParam(OpenHRP::GazeControllerService::gazeParam& i_param_) {
    if(transition_count_ = -1) {
        i_param_.controller_mode = OpenHRP::GazeControllerService::MODE_GAZE;
    } else if (transition_count_ = 0) {
        i_param_.controller_mode = OpenHRP::GazeControllerService::MODE_IDLE;
    } else {
        i_param_.controller_mode = OpenHRP::GazeControllerService::MODE_TRANSITION;
    }
    i_param_.base_type = base_type_;
    i_param_.use_sh_base_pos_rpy = use_sh_base_pos_rpy_;
    i_param_.linkname = target_name_.c_str();

    return true;
}

bool GazeController::waitGazeControllerTransition() {

    if (transition_count_ != 0) {
        while (transition_count_ != 0) usleep(1000);
        return true;
    }
    return false;
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
