#!/usr/bin/env python3
import modo
import math

def lookAt( directionVector = modo.Vector3(1,0,0), 
            upVector        = modo.Vector3(0,1,0), 
            axes            = 'xyz', 
            rotOrder        = 'zxy', 
            asDegrees       = True,
            offsetMatrix    = None):
    
    '''Retuns an euler rotation with x aligned to the given direction vector and y to the optional up vector.
    :param Vector3    directionVector: Direction to aim at
    :param Vector3    upVector:        Direction the twist of the rotation should aim at (what is considered up)
    :param basestring rotOrder:        Rotation order to use for the resulting euler rotation.
    :param bool       asDegrees:       Outputs the rotation as degrees if True, as radians otherwise.
    :param Matrix4    offsetMatrix:    By default x will point in the direction and y is up. This rotational offset matrix can be used to adjust the axes
    '''
    
    directionVector.normalize()
    upVector.normalize()

    # Get up and normal vectors perpendicular to the direction vector by crossing
    normalVector = directionVector.cross(upVector).normal() 
    upVector = normalVector.cross( directionVector ).normal()

    # Create a transformation matrix from the three vectors
    matrix = modo.Matrix4( (directionVector.values, upVector.values, normalVector.values) )
    
    # Apply offset matrix in
    matrix = offsetMatrix * matrix

    # Return as euler rotation values
    return matrix.asEuler(degrees=asDegrees, order=rotOrder)


# Example usage, creating locators:
scene = modo.Scene()

directionVector = modo.Vector3( 0.123, 0.456, 0.789 )
directionLocator = scene.addItem('locator', name='Direction')
directionLocator.position.set(directionVector)

upVector = modo.Vector3(0.34, -0.84, -0.16)
upVectorLocator = scene.addItem('locator', name='UpVector')
upVectorLocator.position.set(upVector)

rotationLocator = scene.addItem('locator', name='Result')

# This method is only available in 901 SP3
#offsetMatrix = modo.Matrix4.fromEuler((0,math.radians(90.0),0))

# ... so I use a quaternion to create the matrix instead now to offset the rotation
# so that -Z points to the given direction
quat = modo.Quaternion()
quat.setAxisAngle( (0,1,0), math.radians(90.0) )
offsetMatrix = quat.toMatrix4()

eulerRotation = lookAt(directionVector, upVector, axes='yxz', offsetMatrix=offsetMatrix)
rotationLocator.rotation.set( eulerRotation, degrees=True)