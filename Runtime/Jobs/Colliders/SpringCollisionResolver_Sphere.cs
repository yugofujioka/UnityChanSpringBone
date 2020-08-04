﻿using UnityEngine;

namespace Unity.Animations.SpringBones.Jobs
{
    public static partial class SpringCollisionResolver
    {
        public static bool ResolveSphere
        (
            SpringColliderProperties sphere,
            SpringColliderComponents transform,
            Vector3 headPosition,
            ref Vector3 tailPosition,
            ref Vector3 hitNormal,
            float tailRadius
        )
        {
            var worldToLocal = transform.worldToLocalMatrix;
            var localTailPosition = worldToLocal.MultiplyPoint3x4(tailPosition);
            var localTailSqrDistance = localTailPosition.sqrMagnitude;

            var combinedRadius = sphere.radius + tailRadius;
            if (localTailSqrDistance >= combinedRadius * combinedRadius)
            {
                // Not colliding
                return false;
            }

            var localHeadPosition = worldToLocal.MultiplyPoint3x4(headPosition);
            var localHeadSqrDistance = localHeadPosition.sqrMagnitude;

            if (localHeadSqrDistance <= sphere.radius * sphere.radius) {
                // The head is inside the sphere, so just try to push the tail out
                localTailPosition = localTailPosition.normalized * combinedRadius;
            } else {
                var localHeadRadius = (localTailPosition - localHeadPosition).magnitude;
                if (ComputeIntersection_Sphere(
                    localHeadPosition, localHeadRadius,
                    Vector3.zero, combinedRadius,
                    out var intersection)) {
                    localTailPosition = ComputeNewTailPosition_Sphere(intersection, localTailPosition);
                }
            }

            var localToWorld = transform.localToWorldMatrix;
            tailPosition = localToWorld.MultiplyPoint3x4(localTailPosition);
            hitNormal = Vector3.Normalize(localToWorld.MultiplyVector(localTailPosition));

            return true;
        }

        // http://mathworld.wolfram.com/Sphere-SphereIntersection.html
        private static bool ComputeIntersection_Sphere
        (
            Vector3 originA,
            float radiusA,
            Vector3 originB,
            float radiusB,
            out Intersection intersection
        )
        {
            var aToB = originB - originA;
            var dSqr = aToB.sqrMagnitude;
            var d = Mathf.Sqrt(dSqr);
            if (d <= 0f)
            {
                intersection = new Intersection();
                return false;
            }

            var radiusASqr = radiusA * radiusA;
            var radiusBSqr = radiusB * radiusB;

            // Assume a is at the origin and b is at (d, 0 0)
            var denominator = 0.5f / d;
            var subTerm = dSqr - radiusBSqr + radiusASqr;
            var x = subTerm * denominator;
            var squaredTerm = subTerm * subTerm;
            var intersectionRadius = Mathf.Sqrt(4f * dSqr * radiusASqr - squaredTerm) * denominator;

            var upVector = aToB / d;
            var origin = originA + x * upVector;

            intersection = new Intersection
            {
                origin = origin,
                radius = intersectionRadius,
                upVector = upVector
            };

            return true;
        }

        private static Vector3 ComputeNewTailPosition_Sphere(Intersection intersection, Vector3 tailPosition)
        {
            // http://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point
            // Project child's position onto the plane
            var newTailPosition = tailPosition
                - Vector3.Dot(intersection.upVector, tailPosition - intersection.origin) * intersection.upVector;
            var v = newTailPosition - intersection.origin;
            var newPosition = intersection.origin + intersection.radius * v.normalized;
            return newPosition;
        }        
    }
}