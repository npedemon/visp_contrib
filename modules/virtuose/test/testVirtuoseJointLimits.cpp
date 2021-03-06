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
  \example testVirtuoseJointLimits.cpp

  Test Haption Virtuose SDK wrapper.
*/

#include <visp3/core/vpTime.h>
#include <visp3/virtuose/vpVirtuose.h>
#include <unistd.h>


void CallBackVirtuose(VirtContext VC, void* ptr)
{
  (void) VC;
  vpVirtuose* p_virtuose=(vpVirtuose*)ptr;

  float maxQ[6] = {0.7811045051,  -0.07668215036,  2.481732368,  2.819076777,  1.044736624,  2.687076807};
  float minQ[6] ={-0.8011951447, -1.648244739, 0.7439950705, -3.022218227, -1.260564089, -2.054088593};
  int numJoint = 6;

//  vpColVector currentQ(numJoint, 0);
  vpColVector feedbackRegion(numJoint, 0);
  vpColVector forceFeedback(numJoint, 0);

  int feedbackRegionFactor = 10;
  float saturationForce[6] = {5,5,5,2.5,2.5,2.5};

  for (int iter=0; iter<numJoint; iter++)
    feedbackRegion[iter] = (maxQ[iter] - minQ[iter])/feedbackRegionFactor;

  for (unsigned int step=0; step<10000; step++)
  {
    vpColVector currentQ = p_virtuose->getArticularPosition();

    // force feedback definition
    for (int iter=0; iter<numJoint; iter++){
      if (currentQ[iter] >= (maxQ[iter] - feedbackRegion[iter]))
      {
        forceFeedback[iter] = -saturationForce[iter]*pow((currentQ[iter] -maxQ[iter] + feedbackRegion[iter])/feedbackRegion[iter],2);
        std::cout << "WARNING! Getting close to the maximum joint limit. Joint #" << iter+1 << std::endl;
      }
      else if (currentQ[iter] <= (minQ[iter] + feedbackRegion[iter]))
      {
        forceFeedback[iter] = saturationForce[iter]*pow((minQ[iter] + feedbackRegion[iter] - currentQ[iter])/feedbackRegion[iter],2);
        std::cout << "WARNING! Getting close to the minimum joint limit. Joint #" << iter+1 << std::endl;
      }
      else
      {
        forceFeedback[iter] = 0;
        std::cout << "Safe zone" << std::endl;
      }
    }

    // Printing force feedback
    //    std::cout << "Force feedback: " << forceFeedback.t() << std::endl;

    // Set force feedback
    p_virtuose->setArticularForce(forceFeedback);
  }

  return;
}

int main()
{
#if defined(VISP_HAVE_VIRTUOSE)
  try {
    vpVirtuose virtuose;
    virtuose.setIpAddress("localhost#5000");
    virtuose.setVerbose(true);
    virtuose.setPowerOn(1);

    float period = 0.001;
    virtuose.setTimeStep(period);

    // Articular feedback for joint limit
    // setArticularForce only works in COMMAND_TYPE_ARTICULAR_IMPEDANCE.
    virtuose.setCommandType(COMMAND_TYPE_ARTICULAR_IMPEDANCE);

    // -----------------------------------------------------------
    // Code to obtain (experimentally) the Virtuose joint limits
    // -----------------------------------------------------------

    /*

    // Move the Virtuose in all its workspace while running this code

    vpColVector joints(6);
    vpColVector max_joint(6,-1000);
    vpColVector min_joint(6,1000);

    for(unsigned int iter=0; iter<10000; iter++) {

      virtuose.getArticularPosition(joints);

      for(unsigned int i=0; i<6; i++) {
        if (joints[i] > max_joint[i])
            max_joint[i] = joints[i];
        if (joints[i] < min_joint[i])
          min_joint[i] = joints[i];
      }

      // Printing joint values
      std::cout << "Joint values: " << joints.t() << std::endl;

      vpTime::wait(10);

    }

    std::cout << "Max Joint values: " << max_joint.t() << std::endl;
    std::cout << "Min Joint values: " << min_joint.t() << std::endl;


    // Best Result (small errors are to be expected)
    // Max Joint values: 0.7811045051  -0.07668215036  2.481732368  2.819076777  1.044736624  2.687076807
    //  Min Joint values: -0.8011951447  -1.648244739  0.7439950705  -3.022218227  -1.260564089  -2.054088593
*/

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
    sleep(1);

    virtuose.setPowerOn(0);
    std::cout << "The end" << std::endl;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }


#else
  std::cout << "You should install Virtuose SDK to use this binary..." << std::endl;
#endif
}

