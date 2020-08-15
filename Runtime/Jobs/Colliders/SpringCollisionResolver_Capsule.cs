using UnityEngine;
using Unity.Mathematics;

namespace Unity.Animations.SpringBones.Jobs
{
    // Up is y-axis
    public static partial class SpringCollisionResolver
    {
        public static bool ResolveCapsule
        (
            SpringColliderProperties capsule,
            SpringColliderComponents transform,
            float3 moverHeadPosition, 
            ref float3 moverPosition, 
            ref float3 hitNormal,
            float moverRadius
        )
        {
            const float RadiusThreshold = 0.0001f;
            if (capsule.radius <= RadiusThreshold)
                return false;

            var worldToLocal = transform.worldToLocalMatrix;
            var radiusScale = math.length(math.rotate(worldToLocal, new float3(1f, 0f, 0f)));

            // Lower than start cap
            var localHeadPosition = math.transform(worldToLocal, moverHeadPosition);
            var localMoverPosition = math.transform(worldToLocal, moverPosition);
            var localMoverRadius = moverRadius * radiusScale;

            var moverIsAboveTop = localMoverPosition.y >= capsule.height;
            var useSphereCheck = (localMoverPosition.y <= 0f) | moverIsAboveTop;
            float combinedRadius;
            
            if (useSphereCheck)
            {
                var sphereOrigin = new float3(0f, moverIsAboveTop ? capsule.height : 0f, 0f);
                combinedRadius = localMoverRadius + capsule.radius;
                var dist = localMoverPosition - sphereOrigin;
                if (math.dot(dist, dist) >= combinedRadius * combinedRadius)
                {
                    // Not colliding
                    return false;
                }

                var originToHead = localHeadPosition - sphereOrigin;
                var isHeadEmbedded = math.dot(originToHead, originToHead) <= capsule.radius * capsule.radius;
                
                if (isHeadEmbedded)
                {
                    // The head is inside the sphere, so just try to push the tail out
                    var localHitNormal = math.normalize(localMoverPosition - sphereOrigin);
                    localMoverPosition = sphereOrigin + localHitNormal * combinedRadius;
                    var localToWorld = transform.localToWorldMatrix;
                    moverPosition = math.transform(localToWorld, localMoverPosition);
                    hitNormal = math.normalize(math.rotate(localToWorld, localHitNormal));
                    return true;
                }

                var localHeadRadius = math.length(localMoverPosition - localHeadPosition);
                if (ComputeIntersection_Sphere(
                    localHeadPosition, localHeadRadius,
                    sphereOrigin, combinedRadius,
                    out var intersection))
                {
                    localMoverPosition = ComputeNewTailPosition_Sphere(intersection, localMoverPosition);
                    var localToWorld = transform.localToWorldMatrix;
                    moverPosition = math.transform(localToWorld, localMoverPosition);
                    var localHitNormal = math.normalize(localMoverPosition - sphereOrigin);
                    hitNormal = math.normalize(math.rotate(localToWorld, localHitNormal));
                }

                return true;
            }

            var originToMover = new float2(localMoverPosition.x, localMoverPosition.z);
            combinedRadius = capsule.radius + localMoverRadius;
            var collided = math.dot(originToMover, originToMover) <= combinedRadius * combinedRadius;
            if (collided)
            {
                var normal = math.normalize(originToMover);
                originToMover = combinedRadius * normal;
                var newLocalMoverPosition = new float3(originToMover.x, localMoverPosition.y, originToMover.y);
                var localToWorld = transform.localToWorldMatrix;
                moverPosition = math.transform(localToWorld, newLocalMoverPosition);
                hitNormal = math.normalize(math.rotate(localToWorld, new float3(normal.x, 0f, normal.y)));
            }

            return collided;
        }
    }
}