/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Test for PointGrey FlyCapture SDK wrapper.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testVirtuosePeriodicFunction.cpp

  Test Haption Virtuose SDK wrapper.
*/

#include <visp3/core/vpTime.h>
#include <visp3/virtuose/vpVirtuose.h>
#include <unistd.h>

//pthread_mutex_t mutex_position;
//vpPoseVector position;

void CallBackVirtuose(VirtContext VC, void* ptr)
{
  (void) VC;
  vpVirtuose* p_virtuose=(vpVirtuose*)ptr;

  vpPoseVector localPose = p_virtuose->getPosition();
  std::cout << localPose.t() << std::endl;

//  pthread_mutex_lock(&mutex_position);
//  position = localPose;
//  pthread_mutex_unlock(&mutex_position);

  return;
}

int main()
{
#if defined(VISP_HAVE_VIRTUOSE)
  try {
    vpVirtuose virtuose;
//    virtuose.setIpAddress("localhost#5000");
    virtuose.setVerbose(true);
//    virtuose.init();
//    virtuose.setPowerOn(1);

    virtuose.setCommandType(COMMAND_TYPE_ARTICULAR_IMPEDANCE);


    float period = 0.001;
    virtuose.setTimeStep(period);


//    pthread_mutex_init(&mutex_position,NULL);

    virtuose.setPeriodicFunction(CallBackVirtuose,period,virtuose);
    virtuose.startPeriodicFunction();

    int counter = 0;
    bool swtch = true;

    while(swtch){
      if (counter>=2)
      {
        virtuose.stopPeriodicFunction();
        swtch = false;
      }

      counter++;
      sleep(1);
    }
//    sleep(1);

    virtuose.setPowerOn(0);
//    pthread_mutex_destroy(&mutex_position);
//    std::cout << "The end" << std::endl;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }


#else
  std::cout << "You should install Virtuose SDK to use this binary..." << std::endl;
#endif
}

