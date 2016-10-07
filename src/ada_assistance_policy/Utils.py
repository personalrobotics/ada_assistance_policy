import numpy as np
import tf.transformations as transmethods


def RotationMatrixDistance(pose1, pose2):
  quat1 = transmethods.quaternion_from_matrix(pose1)
  quat2 = transmethods.quaternion_from_matrix(pose2)
  return QuaternionDistance(quat1, quat2)


def QuaternionDistance(quat1, quat2):
  quat_between = transmethods.quaternion_multiply(  quat2, transmethods.quaternion_inverse(quat1) )
  return AngleFromQuaternionW(quat_between[-1])

def AngleFromQuaternionW(w):
  w = min(0.9999999, max(-0.999999, w))
  phi = 2.*np.arccos(w)
  return min(phi, 2.* np.pi - phi)


def ApplyTwistToTransform(twist, transform, time=1.):
#  transform[0:3,3] += time * twist[0:3]
#  
#  quat = transmethods.quaternion_from_matrix(transform)
#  quat_after_angular = ApplyAngularVelocityToQuaternion(twist[3:], quat, time)
#  transform[0:3, 0:3] = transmethods.quaternion_matrix(quat_after_angular)[0:3, 0:3]

  transform[0:3,3] += time * twist[0:3]

  angular_velocity = twist[3:]
  angular_velocity_norm = np.linalg.norm(angular_velocity)
  if angular_velocity_norm > 1e-3:
    angle = time*angular_velocity_norm
    axis = angular_velocity/angular_velocity_norm
    transform[0:3,0:3] = np.dot(transmethods.rotation_matrix(angle, axis), transform)[0:3,0:3]

  return transform

def ApplyAngularVelocityToQuaternion(angular_velocity, quat, time=1.):
  angular_velocity_norm = np.linalg.norm(angular_velocity)
  angle = time*angular_velocity_norm
  axis = angular_velocity/angular_velocity_norm

  #angle axis to quaternion formula
  quat_from_velocity = np.append(np.sin(angle/2.)*axis, np.cos(angle/2.))

  return transmethods.quaternion_multiply(quat_from_velocity, quat)


