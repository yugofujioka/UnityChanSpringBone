using System.Diagnostics.Contracts;
using UnityEngine;
using Unity.Mathematics;

namespace Unity.Animations.SpringBones.Jobs {
    [System.Serializable]
    public struct AngleLimitComponent {
        public bool active;
        public float min;
        public float max;

        private static float ComputeFalloff(float value, float range) {
            const float Threshold = 0.0001f;
            if (math.abs(range) <= Threshold) { return 0f; }

            var normalizedValue = value / range;
            normalizedValue = math.clamp(normalizedValue, 0f, 1f);
            return math.min(normalizedValue, math.sqrt(normalizedValue));
        }

        // Returns true if exceeded bounds
        [Pure]
        public float3 ConstrainVector
        (
            float3 target,
            float3 basisSide,
            float3 basisUp,
            float3 basisForward,
            float springStrength,
            float deltaTime
        ) {
            var upProjection = Project(target, basisUp);
            var projection = target - upProjection;
            var projectionMagnitude = math.length(projection);
            var originalSine = math.dot(projection / projectionMagnitude, basisSide);
            // The above math might have a bit of floating point error 
            // so clamp the sine value into a valid range so we don't get NaN later
            originalSine = math.clamp(originalSine, -1f, 1f);

            // Use soft limits based on Hooke's Law to reduce jitter,
            // then apply hard limits
            var newAngle = math.degrees(math.asin(originalSine));
            var acceleration = -newAngle * springStrength;
            newAngle += acceleration * deltaTime * deltaTime;

            var minAngle = min;
            var maxAngle = max;
            //var preClampAngle = newAngle;
            newAngle = math.clamp(newAngle, minAngle, maxAngle);

            // Apply falloff
            var curveLimit = (newAngle < 0f) ? minAngle : maxAngle;
            newAngle = ComputeFalloff(newAngle, curveLimit) * curveLimit;

            var radians = math.radians(newAngle);
            var newProjection = math.sin(radians) * basisSide + math.cos(radians) * basisForward;
            newProjection *= projectionMagnitude;

            return newProjection + upProjection;
        }

        public static float3 Project(float3 vector, float3 onNormal) {
            float sqrMag = math.dot(onNormal, onNormal);
            // Burst対応の際にMathf.Epsilonを扱うと怒られる
            // float3.kEpsilonの1E-05は荒い気もするが元々float3演算なのでfloat3の定義に合わせる
            //if (sqrMag < Mathf.Epsilon)
            const float kEpsilon = 1E-05F;
            if (sqrMag < kEpsilon)
                return float3.zero;
            else {
                var dot = math.dot(vector, onNormal);
                return new float3(onNormal.x * dot / sqrMag,
                    onNormal.y * dot / sqrMag,
                    onNormal.z * dot / sqrMag);
            }
        }
    }
}
