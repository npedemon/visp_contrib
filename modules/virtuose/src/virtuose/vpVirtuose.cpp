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

/*!
  \file vpVirtuose.cpp
  \brief Wrapper over Haption Virtuose SDK to control haptic devices.
*/

#include <visp3/core/vpException.h>
#include <visp3/virtuose/vpVirtuose.h>

#ifdef VISP_HAVE_VIRTUOSE

/*!
 * Default constructor.
 * Set command type to virtual mechanism by default.
 */
vpVirtuose::vpVirtuose()
  : m_virtContext(NULL), m_ip("localhost#5000"), m_verbose(false),
    m_apiMajorVersion(0), m_apiMinorVersion(0),
    m_ctrlMajorVersion(0), m_ctrlMinorVersion(0),
    m_typeCommand(COMMAND_TYPE_IMPEDANCE), m_indexType(INDEXING_ALL),
    m_is_init(false)
{
  virtAPIVersion(&m_apiMajorVersion, &m_apiMinorVersion);
  std::cout << "API version: " << m_apiMajorVersion << "." << m_apiMinorVersion << std::endl;
}

/*!
 * Default destructor that delete the VirtContext object.
 */
vpVirtuose::~vpVirtuose()
{
  if (m_virtContext != NULL) {
    virtClose(m_virtContext);
    m_virtContext = NULL;
  }
}

/*!
 * Add a force to be applied to the virtuose (impedance effort).
 * This function works in every mode.
 * \param force : Is 6 component dynamic tensor (three forces and three torques) wrt virtuose end-effector
 * and is expressed in the coordinates of the base frame.
 */
void vpVirtuose::addForce (vpColVector &force)
{
  if (force.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot apply a force feedback (dim %d) to the haptic device that is not 6-dimension",
                      force.size()));
  }

  init();

  float virtforce[6];
  for(unsigned int i=0; i<6; i++)
    virtforce[i] = force[i];

  if (virtAddForce(m_virtContext, virtforce)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtAddForce: error code %d", err));
  }
}

/*!
 * Activate or desactivate force feedback.
 * \param enable : 1 to activate (system's default value), 0 to desactivate.
 */
void vpVirtuose::enableForceFeedback (int enable)
{
  init();

  if (virtEnableForceFeedback(m_virtContext, enable)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtEnableForceFeedback(): error code %d", err));
  }
}

/*!
 * Return the 6 joint values of the virtuose.
 */
vpColVector vpVirtuose::getArticularPosition() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpColVector articularPosition(6,0);

  float articular_position_[6];
  if (virtGetArticularPosition(m_virtContext, articular_position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetArticularPosition(): error code %d", err));
  }

  for(unsigned int i=0; i<6; i++)
    articularPosition[i] = articular_position_[i];

  return articularPosition;
}


/*!
 * Return the 6 joint velocities of the virtuose.
  */
vpColVector vpVirtuose::getArticularVelocity() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpColVector articularVelocity(6,0);
  float articular_velocity_[6];
  if (virtGetArticularSpeed(m_virtContext, articular_velocity_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetArticularSpeed: error code %d", err));
  }

  for(unsigned int i=0; i<6; i++)
    articularVelocity[i] = articular_velocity_[i];

  return articularVelocity;

}

/*!
 * Return avatar position expressed in the environment reference frame.
 * With respect to the function getPosition(),
 * getAvatarPosition() takes into account offsets
 * and motor scale factors.
 */
vpPoseVector vpVirtuose::getAvatarPosition() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  float position_[7];
  vpPoseVector position;
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetAvatarPosition(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetAvatarPosition: error code %d", err));
  }
  else
  {
    for (int i=0; i<3; i++)
      translation[i] = position_[i];
    for (int i=0; i<4; i++)
      quaternion[i] = position_[3+i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);

    return position;
  }
}

/*!
 * Return the current position of the base frame
 * with respect to the observation reference frame.
 *
 * \sa getObservationFrame()
 */
vpPoseVector vpVirtuose::getBaseFrame() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpPoseVector position;
  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetBaseFrame(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetBaseFrame: error code %d", err));
  }
  else
  {
    for (int i=0; i<3; i++)
      translation[i] = position_[i];
    for (int i=0; i<4; i++)
      quaternion[i] = position_[3+i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);

    return position;
  }
}

/*!
 * Return the command type.
 */
VirtCommandType vpVirtuose::getCommandType () const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  VirtCommandType type;

  if (virtGetCommandType(m_virtContext, &type)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetCommandType: error code %d", err));
  }
  return type;
}

/*!
 * Return the status of DeadMan sensor : 1 if the sensor is ON (a user is holding the handle) and 0 if the sensor is OFF (no user detected).
 */
bool vpVirtuose::getDeadMan () const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  int deadman;
  virtGetDeadMan(m_virtContext, &deadman);
  return (deadman ? true : false);
}

/*!
 * Return force tensor to be applied to the attached object.
 */
vpColVector vpVirtuose::getForce() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpColVector force(6,0);
  float force_[6];
  if (virtGetForce(m_virtContext, force_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetForce: error code %d", err));
  }

  for(unsigned int i=0; i<6; i++)
    force[i] = force_[i];
  return force;
}

/*!
 * Get the current position of the observation reference frame
 * with respect to the base frame.
 *
 * \sa getBaseFrame()
 */
vpPoseVector vpVirtuose::getObservationFrame () const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpPoseVector position;
  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetObservationFrame(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetObservationFrame: error code %d", err));
  }
  else
  {
    for (int i=0; i<3; i++)
      translation[i] = position_[i];
    for (int i=0; i<4; i++)
      quaternion[i] = position_[3+i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);
  }
  return position;
}

/*!
 * Return device end effector position expressed in the base frame.
 */
vpPoseVector vpVirtuose::getPhysicalPosition() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpPoseVector position;
  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetPhysicalPosition(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetPhysicalPosition: error code %d", err));
  }
  else
  {
    for (int i=0; i<3; i++)
      translation[i] = position_[i];
    for (int i=0; i<4; i++)
      quaternion[i] = position_[3+i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);
  }
  return position;
}

/*!
 * Return device end effector velocity expressed in the base frame.
 */
vpColVector vpVirtuose::getPhysicalVelocity() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpColVector vel(6,0);
  float speed[6];
  if (virtGetPhysicalSpeed(m_virtContext, speed)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling virtGetPhysicalSpeed: error code %s",
                      virtGetErrorMessage(err)));
  }
  for(unsigned int i=0; i<6; i++)
    vel[i] = speed[i];
  return vel;
}

/*!
 * Return position of the virtuose (or the object attached to it) wrt the environment frame.
 */
vpPoseVector vpVirtuose::getPosition() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpPoseVector position;
  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  if (virtGetPosition(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtGetPosition: error code %d", err));
  }
  else
  {
    for (int i=0; i<3; i++)
      translation[i] = position_[i];
    for (int i=0; i<4; i++)
      quaternion[i] = position_[3+i];

    vpThetaUVector thetau(quaternion);

    position.buildFrom(translation, thetau);
  }
  return position;
}

/*!
 * Return status of the motors : true if motors are ON, else false.
 */
bool vpVirtuose::getPower () const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  int power;
  virtGetPowerOn(m_virtContext, &power);
  return (power ? true : false);
}


/*!
 * Return device end effector velocity.
 */
vpColVector vpVirtuose::getVelocity() const
{
  if (!m_is_init) {
    throw(vpException(vpException::fatalError, "Device not initialized. Call init()."));
  }

  vpColVector vel(6,0);
  float speed[6];
  if (virtGetSpeed(m_virtContext, speed)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Cannot get haptic device velocity: %s",
                      virtGetErrorMessage(err)));
  }
  for(unsigned int i=0; i<6; i++)
    vel[i] = speed[i];
  return vel;
}

/*!
 * Initialize virtuose device opening the connection to the device and setting the default command type.
 * If the device is already initialized, a call to init() does nothing.
 */
void vpVirtuose::init()
{
  if (! m_is_init) {
    m_virtContext = virtOpen(m_ip.c_str());

    if (m_virtContext == NULL) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Cannot open haptic device: %s",
                        virtGetErrorMessage(err)));
    }

    if (virtGetControlerVersion(m_virtContext, &m_ctrlMajorVersion, &m_ctrlMinorVersion)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Cannot get haptic device controller version: %s",
                        virtGetErrorMessage(err)));
    }

    if (m_verbose) {
      std::cout << "Controller version: " << m_ctrlMajorVersion << "." << m_ctrlMinorVersion << std::endl;
    }
    if (virtSetCommandType(m_virtContext, m_typeCommand)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError, "Cannot set haptic device command type: %s",
                        virtGetErrorMessage(err)));
    }
    m_is_init = true;
  }
}

/*!
 * Set articular force.
 * Works in mode COMMAND_TYPE_ARTICULAR_IMPEDANCE that need to be set with setCommandType().
 * \param articularForce :
 */
void vpVirtuose::setArticularForce (const vpColVector &articularForce)
{
  init();

  if (articularForce.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot apply an articular force feedback (dim %d) to the haptic device that is not 6-dimension",
                      articularForce.size()));
  }

  float articular_force[6];
  for(unsigned int i=0; i<6; i++)
    articular_force[i] = articularForce[i];

  if (virtSetArticularForce(m_virtContext, articular_force)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetArticularForce: error code %d", err));
  }
}

/*!
 * Send articular (joint) position command to the virtuose.
 * Works in COMMAND_TYPE_ARTICULAR mode that need to be set with setCommandType().
 * \param articularPosition : Six dimension joint position vector.
 */
void vpVirtuose::setArticularPosition (const vpColVector &articularPosition)
{
  init();

  if (articularPosition.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot send an articular position command (dim %d) to the haptic device that is not 6-dimension",
                      articularPosition.size()));
  }

  float articular_position[6];
  for(unsigned int i=0; i<6; i++)
    articular_position[i] = articularPosition[i];

  if (virtSetArticularPosition(m_virtContext, articular_position)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetArticularPosition: error code %d", err));
  }
}

/*!
 * Send articular (joint) velocity command to the virtuose.
 * Works in COMMAND_TYPE_ARTICULAR mode  that need to be set with setCommandType().
 * \param articularVelocity : Six dimension joint velocity vector.
 */
void vpVirtuose::setArticularVelocity (const vpColVector &articularVelocity)
{
  init();

  if (articularVelocity.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot send an articular velocity command (dim %d) to the haptic device that is not 6-dimension",
                      articularVelocity.size()));
  }

  float articular_velocity[6];
  for(unsigned int i=0; i<6; i++)
    articular_velocity[i] = articularVelocity[i];

  if (virtSetArticularSpeed(m_virtContext, articular_velocity)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetArticularVelocity: error code %d", err));
  }
}

/*!
 * Set base frame with respect to the observation frame
 * \param position : Position of the base frame.
 *
 * \sa setObservationFrame()
 */
void vpVirtuose::setBaseFrame (const vpPoseVector &position)
{
  init();

  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  position.extract(translation);
  position.extract(quaternion);

  for (int i=0; i<3; i++)
    position_[i] = translation[i];
  for (int i=0; i<4; i++)
    position_[3+i] = quaternion[i];

  if (virtSetBaseFrame(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetBaseFrame: error code %d", err));
  }
}

/*!
 * Set the command type.
 * \param type : Possible values are COMMAND_TYPE_NONE, COMMAND_TYPE_IMPEDANCE, COMMAND_TYPE_ARTICULAR_IMPEDANCE, COMMAND_TYPE_VIRTMECH
 * - COMMAND_TYPE_NONE :
 * - COMMAND_TYPE_IMPEDANCE :
 */
void vpVirtuose::setCommandType(const VirtCommandType &type)
{
  init();

  if (m_typeCommand != type) {
    m_typeCommand = type;

    if (virtSetCommandType(m_virtContext, m_typeCommand)) {
      int err = virtGetErrorCode(m_virtContext);
      throw(vpException(vpException::fatalError,
                        "Error calling virtSetCommandType: error code %d", err));
    }
  }
}

/*!
 * Set the force to be applied by the Virtuose.
 * Works in impedance mode that could be set with setCommandType().
 * \param force : Force vector that represents a dynamic tensor with 6 components.
 */
void vpVirtuose::setForce (const vpColVector &force)
{
  init();

  if (force.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot apply a force feedback (dim %d) to the haptic device that is not 6-dimension",
                      force.size()));
  }

  float virtforce[6];
  for(unsigned int i=0; i<6; i++)
    virtforce[i] = force[i];

  if (virtSetForce(m_virtContext, virtforce)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetForce: error code %d", err));
  }
}

/*!
 * Set force factor.
 * \param forceFactor : Force factor scale applied to the force torque tensor set by setForce().
 */
void vpVirtuose::setForceFactor (const float &forceFactor)
{
  init();

  if (virtSetForceFactor(m_virtContext, forceFactor)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetForceFactor: error code %d", err));
  }
}

/*!
 * Set indexing (offset) mode.
 * \param type : Possible choices: INDEXING_ALL, INDEXING_TRANS (only translations),
 * INDEXING_NONE
 */
void vpVirtuose::setIndexingMode (const VirtIndexingType &type)
{
  init();

  if (virtSetIndexingMode(m_virtContext, type)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling setIndexingMode: error code %d", err));
  }
}

/*!
 * Set observation frame with respect to the environment frame.
 * \param position : Position of the observation frame.
 *
 * \sa setBaseFrame()
 */
void vpVirtuose::setObservationFrame (const vpPoseVector &position)
{
  init();

  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  position.extract(translation);
  position.extract(quaternion);

  for (int i=0; i<3; i++)
    position_[i] = translation[i];
  for (int i=0; i<4; i++)
    position_[3+i] = quaternion[i];

  if (virtSetObservationFrame(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetObservationFrame: error code %d", err));
  }
}

/*!
 * Register the periodic function. This function could be started using
 * startLoop() and stopped using stopLoop().
 * \param CallBackVirt :
 * \param period :
 * \param virtuose :
 */
void vpVirtuose::setPeriodicFunction(VirtPeriodicFunction CallBackVirt, float &period, vpVirtuose &virtuose)
{
  init();

  if (virtSetPeriodicFunction(m_virtContext, CallBackVirt, &period, &virtuose)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetPeriodicFunction: error code %d", err));
  }
}

/*!
 * Modify the current control position.
 * \param position :
 */
void vpVirtuose::setPosition (vpPoseVector &position)
{
  init();

  float position_[7];
  vpTranslationVector translation;
  vpQuaternionVector quaternion;

  position.extract(translation);
  position.extract(quaternion);

  for (int i=0; i<3; i++)
    position_[i] = translation[i];
  for (int i=0; i<4; i++)
    position_[3+i] = quaternion[i];

  if (virtSetPosition(m_virtContext, position_)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetPosition: error code %d", err));
  }
}

/*!
 * Set power On
 * \param power : When set to 1 allows to turn on the motors. When 0 turns them off.
 */
void vpVirtuose::setPowerOn (const int &power)
{
  init();

  if (virtSetPowerOn(m_virtContext, power)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetPowerOn: error code %d", err));
  }
}

/*!
 * Set saturation values of the force feedback
 * \param forceLimit : Value expressed in N.
 * \param torqueLimit : Value expressed in Nm.
 */
void vpVirtuose::setSaturation(const float &forceLimit, const float &torqueLimit)
{
  init();

  if (virtSaturateTorque(m_virtContext, forceLimit, torqueLimit)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSaturateTorque: error code %d", err));
  }
}

/*!
 * Set the the simulation time step.
 * \param timeStep :
 */
void vpVirtuose::setTimeStep (const float &timeStep)
{
  init();

  if (virtSetTimeStep(m_virtContext, timeStep)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetTimeStep: error code %d", err));
    //    throw(vpException(vpException::fatalError,
    //                      "Error calling virtSetTimeStep: %s", virtGetErrorMessage(err)));
  }
}

/*!
 * Modify the current control speed.
 * \param velocity : Velocity twist vector, where translations velocities are expressed in
 * m/s and rotation velocities in rad/s.
 */
void vpVirtuose::setVelocity (vpColVector &velocity)
{
  init();

  if (velocity.size() != 6) {
    throw(vpException(vpException::dimensionError,
                      "Cannot set a velocity vector (dim %d) that is not 6-dimension",
                      velocity.size()));
  }

  float speed[6];
  for(unsigned int i=0; i<6; i++)
    speed[i] = velocity[i];

  if (virtSetSpeed(m_virtContext, speed)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError,
                      "Error calling virtSetSpeed: error code %d", err));
  }
}

/*!
 * Set the speed factor.
 * \param velocityFactor : Scale factor applied to the velocities set using setVelocity()
 */
void vpVirtuose::setVelocityFactor (const float &velocityFactor)
{
  init();

  if (virtSetSpeedFactor(m_virtContext, velocityFactor)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling setVelocityFactor: error code %d",
                      err));
  }
}

/*!
 * Start the periodic function set using setPeriodicFunction().
 *
 * \sa stopPeriodicFunction(), setPeriodicFunction()
 */
void vpVirtuose::startPeriodicFunction()
{
  init();

  if (virtStartLoop(m_virtContext)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling startLoop: error code %d",
                      err));
  }
  else
    std::cout << "Haptic loop open." << std::endl;
}

/*!
 * Stop the periodic function set using setPeriodicFunction().
 *
 * \sa startPeriodicFunction(), setPeriodicFunction()
 */
void vpVirtuose::stopPeriodicFunction()
{
  init();

  if (virtStopLoop(m_virtContext)) {
    int err = virtGetErrorCode(m_virtContext);
    throw(vpException(vpException::fatalError, "Error calling stopLoop: error code %d",
                      err));
  }
  else
    std::cout << "Haptic loop closed." << std::endl;
}

#else
// Work around to avoid warning
void dummy_vpVirtuose() {};
#endif


