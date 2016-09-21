// -*-C++-*-
/*!
 * @file  GazeControllerService_impl.h
 * @brief Service implementation header of GazeControllerService.idl
 *
 */
#ifndef GAZECONTROLLERSERVICE_IMPL_H
#define GAZECONTROLLERSERVICE_IMPL_H

#include "hrpsys/idl/GazeControllerService.hh"

using namespace OpenHRP;

class GazeController;

/*
 * Example class implementing IDL interface OpenHRP::GazeControllerService
 */
class GazeControllerService_impl
 : public virtual POA_OpenHRP::GazeControllerService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~OpenHRP_GazeControllerService_impl();

 public:
   // standard constructor
   GazeControllerService_impl();
   virtual ~GazeControllerService_impl();

   // attributes and operations
   ::CORBA::Boolean startGazeController();
   ::CORBA::Boolean startGazeControllerNoWait();
   ::CORBA::Boolean stopGazeController();
   ::CORBA::Boolean stopGazeControllerNoWait();
   ::CORBA::Boolean setGazeControllerParam(const OpenHRP::GazeControllerService::gazeParam& i_param);
   ::CORBA::Boolean getGazeControllerParam(OpenHRP::GazeControllerService::gazeParam_out i_param);
   void waitGazeControllerTransition();

  //
  void gaze(GazeController *i_gaze);
private:
  GazeController *m_gaze;
};

#endif // GAZECONTROLLERSERVICE_IMPL_H
