using UnityEngine;
using Unity.Mathematics;

namespace Unity.Animations.SpringBones.Jobs
{
    // Up is y-axis
    public static partial class SpringCollisionResolver
    {

//        public float3 GetPlaneNormal()
//        {
//            return transform.forward;
//        }

        public static bool ResolvePanel
        (
            SpringColliderProperties panel,
            SpringColliderComponents transform,
            float3 headPosition,
            ref float3 tailPosition,
            ref float3 hitNormal,
            float length,
            float tailRadius
        )
        {
            var localTailPosition = math.transform(transform.localToWorldMatrix, tailPosition);
            
            // Plane transform is z-up. if hence z >= tailRadius, there is no collision.  
            if (localTailPosition.z >= tailRadius)
            {
                return false;
            }

            var localHeadPosition = math.transform(transform.worldToLocalMatrix, headPosition);

            var halfWidth = panel.width / 2f;
            var halfHeight = panel.height /2f;

            var pointOnPlane = math.lerp(localHeadPosition, localTailPosition,
                math.clamp(localHeadPosition.z/(localHeadPosition.z - localTailPosition.z), 0f, 1f));
            
            if (math.abs(pointOnPlane.x) >= halfWidth + tailRadius || 
                math.abs(pointOnPlane.y) >= halfHeight + tailRadius)
            {
                return false;
            }

            // Check edges
            // SpringBone is entirely over plane (only sphere is crossing)
            if (localHeadPosition.z <= 0f && localTailPosition.z <= 0f)
            {
                if (math.abs(localHeadPosition.y) > halfHeight)
                {
                    halfHeight = (localTailPosition.y < 0f) ? -halfHeight : halfHeight;
                    localTailPosition = new float3(localTailPosition.x, halfHeight, localTailPosition.z);
                }
                else if (math.abs(localHeadPosition.x) > halfWidth)
                {
                    halfWidth = (localTailPosition.x < 0f) ? -halfWidth : halfWidth;
                    localTailPosition = new float3(halfWidth, localTailPosition.y, localTailPosition.z);
                }
                else
                {
                    localTailPosition = localHeadPosition;
                    localTailPosition.z = length;
                }
            } 
            
            else {
                if (math.abs(localTailPosition.y) > halfHeight)
                {
                    halfHeight = (localTailPosition.y < 0f) ? -halfHeight : halfHeight;
                    var localNormal = math.normalize(new float3(0f, localTailPosition.y - halfHeight, localTailPosition.z));
                    localTailPosition =
                        new float3(localTailPosition.x, halfHeight, 0f) + tailRadius * localNormal;
                }
                else if (math.abs(localTailPosition.x) > halfWidth)
                {
                    halfWidth = (localTailPosition.x < 0f) ? -halfWidth : halfWidth;
                    var localNormal = math.normalize(new float3(localTailPosition.x - halfWidth, 0f, localTailPosition.z));
                    localTailPosition = new float3(halfWidth, localTailPosition.y, 0f) + tailRadius * localNormal;
                }
                else
                {
                    var newLocalTailPosition = localHeadPosition;
                    if (localHeadPosition.z + length <= tailRadius)
                    {
                        // Bone is completely embedded
                        newLocalTailPosition.z += length;
                    }
                    else
                    {
                        var heightAboveRadius = localHeadPosition.z - tailRadius;
                        var projectionLength =
                            math.sqrt(length * length - heightAboveRadius * heightAboveRadius);
                        var localBoneVector = localTailPosition - localHeadPosition;
                        var projectionVector = new float2(localBoneVector.x, localBoneVector.y);
                        var projectionVectorLength = math.length(projectionVector);
                        if (projectionVectorLength > 0.001f)
                        {
                            var projection = (projectionLength / projectionVectorLength) * projectionVector;
                            newLocalTailPosition = new float3
                            {
                                x = newLocalTailPosition.x + projection.x,
                                y = newLocalTailPosition.y + projection.y,
                                z = newLocalTailPosition.z + tailRadius,
                            };
                        }
                    }
                    localTailPosition = newLocalTailPosition;
                }
            }

            tailPosition = math.transform(transform.localToWorldMatrix, localTailPosition);
            hitNormal = math.normalize(math.transform(transform.localToWorldMatrix, new float3(0, 0, 1f))); 

            return true;
        }

        public static bool ResolvePanelOnAxis
        (
            float3 localHeadPosition,
            ref float3 localTailPosition,
            float localLength,
            float localTailRadius,
            Axis upAxis
        )
        {
            var zIndex = (int) upAxis;
            if (localTailPosition[zIndex] >= localTailRadius)
            {
                return false;
            }

            var newLocalTailPosition = localHeadPosition;
            if (localHeadPosition[zIndex] + localLength <= localTailRadius)
            {
                // Bone is completely embedded
                newLocalTailPosition[zIndex] += localLength;
            }
            else
            {
                var xIndex = (zIndex + 1) % (int) Axis.AxisCount;
                var yIndex = (zIndex + 2) % (int) Axis.AxisCount;

                var heightAboveRadius = localHeadPosition[zIndex] - localTailRadius;
                var projectionLength = math.sqrt(localLength * localLength - heightAboveRadius * heightAboveRadius);
                var localBoneVector = localTailPosition - localHeadPosition;
                var projectionVector = new float2(localBoneVector[xIndex], localBoneVector[yIndex]);
                var projectionVectorLength = math.length(projectionVector);
                if (projectionVectorLength > 0.001f)
                {
                    var projection = (projectionLength / projectionVectorLength) * projectionVector;
                    newLocalTailPosition[xIndex] += projection.x;
                    newLocalTailPosition[yIndex] += projection.y;
                    newLocalTailPosition[zIndex] = localTailRadius;
                }
            }

            localTailPosition = newLocalTailPosition;
            return true;
        }
    }
}