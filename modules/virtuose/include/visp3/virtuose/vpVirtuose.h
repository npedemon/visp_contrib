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
 * Description: Class which enables to project an image in the 3D space
 * and get the view of a virtual camera.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpVirtuose_h_
#define __vpVirtuose_h_

#include <ostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/virtuose/vpConfigVirtuose.h>

#ifdef VISP_HAVE_VIRTUOSE

#include <VirtuoseAPI.h>


/*!
  \file vpVirtuose.h
  \brief Wrapper over Haption Virtuose SDK to control haptic devices.
*/
/*!
  \class vpVirtuose
  \ingroup group_robot_real

  Allows to ..



 * The library defines the following reference frames:
 *     1. The environment frame, corresponding to the origin of the virtual scene; it is specified by the software application independently of the Virtuose API.
 *     2. The observation frame, corresponding generally to the position of the camera; it is defined with respect to environment frame.
 *     3. The base frame, representing the center of the haptic device; it is defined with respect to the observation frame.
 *     4. The tool frame corresponds to the base of the tool fixed at the end of the haptic device, and is defined with respect to the environment frame.
 *     5. The end-effector (avatar) frame corresponds to the position of the user hand on the device, taking into account the geometry of the tool, and is defined with respect to tool frame.
 *
 * The position of the following frames can be defined only once using the API:
 * Base frame (with respect to the observation frame)
 * End-effector frame (with respect to the tool frame)
 *
 * The position of the following frames can be modified dynamically using the API:
 * Observation frame (with respect to the environment frame)
 *
 * The position of the following frame cannot be modified:
 * Tool frame (with respect to the environment frame).
 *
 *
 *
  \code

  \endcode
 */
class VISP_EXPORT vpVirtuose
{
public:
  vpVirtuose();
  ~vpVirtuose();

  void addForce (vpColVector &force);
  void enableForceFeedback (int enable);

  vpColVector getArticularPosition() const;
  vpColVector getArticularVelocity() const;
  vpPoseVector getAvatarPosition() const ;
  vpPoseVector getBaseFrame() const;
  VirtCommandType getCommandType() const;
  bool getDeadMan() const;
  vpColVector getForce() const;
  vpPoseVector getObservationFrame() const;
  vpPoseVector getPhysicalPosition() const;
  vpColVector getPhysicalVelocity() const;
  vpPoseVector getPosition() const;
  bool getPower() const;
  vpColVector getVelocity() const;

  void init();

  void setArticularForce(const vpColVector &articularForce);
  void setArticularPosition(const vpColVector &articularPosition);
  void setArticularVelocity(const vpColVector &articularVelocity);
  void setBaseFrame (const vpPoseVector &position);
  void setCommandType(const VirtCommandType &type);
  void setForce(const vpColVector &force);
  void setForceFactor (const float &forceFactor);
  void setIndexingMode (const VirtIndexingType &type) ;
  /*! Set haptic device ip address and port. Default value is "localhost#5000".*/
  inline void setIpAddress(const std::string &ip) {m_ip = ip;}
  void setObservationFrame (const vpPoseVector &position);
  void setPeriodicFunction(VirtPeriodicFunction CallBackVirt, float &period, vpVirtuose &virtuose);
  void setPosition(vpPoseVector &position);
  void setPowerOn(const int &power);
  void setSaturation(const float &forceLimit, const float &torqueLimit);
  void setTimeStep(const float &timeStep);
  void setVelocity(vpColVector &velocity);
  void setVelocityFactor(const float &velocityFactor);
  /*!
     * Enable/disable verbose mode.
     * \param mode true to enable, false to disable verbose.
     */
  void setVerbose(bool mode) {m_verbose = mode;}

  void startPeriodicFunction();
  void stopPeriodicFunction();

protected:
  VirtContext m_virtContext;
  std::string m_ip;
  bool m_verbose;
  int m_apiMajorVersion;
  int m_apiMinorVersion;
  int m_ctrlMajorVersion;
  int m_ctrlMinorVersion;
  VirtCommandType m_typeCommand;
  VirtIndexingType m_indexType;
  bool m_is_init;
};

#endif
#endif
