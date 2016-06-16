// -*-C++-*-
/*!
 * @file  GazeControllerService_impl.cpp
 * @brief Service implementation code of GazeControllerService.idl
 *
 */

#include "GazeControllerService_impl.h"

/*
 * Example implementational code for IDL interface OpenHRP::GazeControllerService
 */
GazeControllerService_impl::GazeControllerService_impl()
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

}

::CORBA::Boolean GazeControllerService_impl::startGazeControllerNoWait()
{

}

::CORBA::Boolean GazeControllerService_impl::stopGazeController()
{

}

::CORBA::Boolean GazeControllerService_impl::stopGazeControllerNoWait()
{

}

::CORBA::Boolean GazeControllerService_impl::setGazeControllerParam(const OpenHRP::GazeControllerService::gazeParam& i_param)
{

}

::CORBA::Boolean GazeControllerService_impl::getGazeControllerParam(OpenHRP::GazeControllerService::gazeParam_out i_param)
{

}

void GazeControllerService_impl::waitGazeControllerTransition()
{

}

// End of example implementational code
void GazeControllerService_impl::gaze(GazeController *i_gaze)
{
    m_gaze = i_gaze;
}
