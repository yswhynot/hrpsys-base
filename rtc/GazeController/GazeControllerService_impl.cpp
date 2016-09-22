// -*-C++-*-
/*!
 * @file  GazeControllerService_impl.cpp
 * @brief Service implementation code of GazeControllerService.idl
 *
 */

#include "GazeControllerService_impl.h"

//
#include "GazeController.h"

/*
 * Example implementational code for IDL interface OpenHRP::GazeControllerService
 */
GazeControllerService_impl::GazeControllerService_impl() : m_gaze(NULL)
{
  // Please add extra constructor code here.
}


GazeControllerService_impl::~GazeControllerService_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
::CORBA::Boolean GazeControllerService_impl::startGazeController()
{
    m_gaze->startGazeController();
    return true;
}

::CORBA::Boolean GazeControllerService_impl::stopGazeController(CORBA::Double tm, CORBA::Boolean wait)
{
    m_gaze->stopGazeController(tm);
    if (wait) {
        return m_gaze->waitGazeControllerTransition();
    }
    return true;
}

::CORBA::Boolean GazeControllerService_impl::setGazeControllerParam(const OpenHRP::GazeControllerService::gazeParam& i_param)
{
    return m_gaze->setGazeControllerParam(i_param);
}

::CORBA::Boolean GazeControllerService_impl::getGazeControllerParam(OpenHRP::GazeControllerService::gazeParam_out i_param)
{
    i_param = new OpenHRP::GazeControllerService::gazeParam();
    return m_gaze->getGazeControllerParam(*i_param);
}

::CORBA::Boolean GazeControllerService_impl::waitGazeControllerTransition()
{
    return m_gaze->waitGazeControllerTransition();
}

// End of example implementational code
void GazeControllerService_impl::gaze(GazeController *i_gaze)
{
    m_gaze = i_gaze;
}
