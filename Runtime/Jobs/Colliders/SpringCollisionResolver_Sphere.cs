using UnityEngine;
using Unity.Mathematics;

namespace Unity.Animations.SpringBones.Jobs
{
    public static partial class SpringCollisionResolver
    {
        public static bool ResolveSphere
        (
            SpringColliderProperties sphere,
            SpringColliderComponents transform,
            float3 headPosition,
            ref float3 tailPosition,
            ref float3 hitNormal,
            float tailRadius
        )
        {
            var worldToLocal = transform.worldToLocalMatrix;
            var localTailPosition = math.transform(worldToLocal, tailPosition);
            var localTailSqrDistance = math.dot(localTailPosition, localTailPosition);

            var combinedRadius = sphere.radius + tailRadius;
            if (localTailSqrDistance >= combinedRadius * combinedRadius)
            {
                // Not colliding
                return false;
            }

            var localHeadPosition = math.transform(worldToLocal, headPosition);
            var localHeadSqrDistance = math.dot(localHeadPosition, localHeadPosition);

            if (localHeadSqrDistance <= sphere.radius * sphere.radius) {
                // The head is inside the sphere, so just try to push the tail out
                localTailPosition = math.normalize(localTailPosition) * combinedRadius;
            } else {
                var localHeadRadius = math.length(localTailPosition - localHeadPosition);
                if (ComputeIntersection_Sphere(
                    localHeadPosition, localHeadRadius,
                    float3.zero, combinedRadius,
                    out var intersection)) {
                    localTailPosition = ComputeNewTailPosition_Sphere(intersection, localTailPosition);
                }
            }

            var localToWorld = transform.localToWorldMatrix;
            tailPosition = math.transform(localToWorld, localTailPosition);
            hitNormal = math.normalize(math.rotate(localToWorld, localTailPosition));

            return true;
        }

        // http://mathworld.wolfram.com/Sphere-SphereIntersection.html
        private static bool ComputeIntersection_Sphere
        (
            float3 originA,
            float radiusA,
            float3 originB,
            float radiusB,
            out Intersection intersection
        )
        {
            var aToB = originB - originA;
            var dSqr = math.dot(aToB, aToB);
            var d = math.sqrt(dSqr);
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
            var intersectionRadius = math.sqrt(4f * dSqr * radiusASqr - squaredTerm) * denominator;

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

        private static float3 ComputeNewTailPosition_Sphere(Intersection intersection, float3 tailPosition)
        {
            // http://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point
            // Project child's position onto the plane
            var newTailPosition = tailPosition
                - math.dot(intersection.upVector, tailPosition - intersection.origin) * intersection.upVector;
            var v = newTailPosition - intersection.origin;
            var newPosition = intersection.origin + intersection.radius * math.normalize(v);
            return newPosition;
        }        
    }
}