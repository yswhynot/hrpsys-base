// -*-C++-*-
/*!
 * @file  ../AccelerationFilterService_impl.cpp
 * @brief Service implementation code of ../AccelerationFilterService.idl
 *
 */
#include <iostream>
#include "AccelerationFilterService_impl.h"

#include "AccelerationFilter.h"

/*
 * Example implementational code for IDL interface OpenHRP::AccelerationFilterService
 */
AccelerationFilterService_impl::AccelerationFilterService_impl()
{
    // Please add extra constructor code here.
}


AccelerationFilterService_impl::~AccelerationFilterService_impl()
{
    // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
::CORBA::Boolean AccelerationFilterService_impl::resetFilter(OpenHRP::AccelerationFilterService::ControlMode mode)
{
    // Please insert your code here and remove the following warning pragma
    return true;
}



// End of example implementational code

void AccelerationFilterService_impl::setInstance(AccelerationFilter *i_instance)
{
    m_instance = i_instance;
}

