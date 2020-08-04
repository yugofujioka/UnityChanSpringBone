using System.Diagnostics.Contracts;
using UnityEngine;

namespace Unity.Animations.SpringBones.Jobs
{
    [System.Serializable]
    public struct AngleLimitComponent
    {
		// FIXED: fujioka - For IL2CPP
        //public bool active;
		public int active;

        public float min;
        public float max;

        private static float ComputeFalloff(float value, float range)
        {
            const float Threshold = 0.0001f;
            if (Mathf.Abs(range) <= Threshold) { return 0f; }

            var normalizedValue = value / range;
            normalizedValue = Mathf.Clamp01(normalizedValue);
            return Mathf.Min(normalizedValue, Mathf.Sqrt(normalizedValue));
        }

        // Returns true if exceeded bounds
        [Pure]
        public Vector3 ConstrainVector
        (
            Vector3 target,
            Vector3 basisSide,
            Vector3 basisUp,
            Vector3 basisForward,
            float springStrength,
            float deltaTime
        )
        {
            var upProjection = Project(target, basisUp);
            var projection = target - upProjection;
            var projectionMagnitude = projection.magnitude;
            var originalSine = Vector3.Dot(projection / projectionMagnitude, basisSide);
            // The above math might have a bit of floating point error 
            // so clamp the sine value into a valid range so we don't get NaN later
            originalSine = Mathf.Clamp(originalSine, -1f, 1f);

            // Use soft limits based on Hooke's Law to reduce jitter,
            // then apply hard limits
            var newAngle = Mathf.Rad2Deg * Mathf.Asin(originalSine);
            var acceleration = -newAngle * springStrength;
            newAngle += acceleration * deltaTime * deltaTime;

            var minAngle = min;
            var maxAngle = max;
            var preClampAngle = newAngle;
            newAngle = Mathf.Clamp(newAngle, minAngle, maxAngle);

            // Apply falloff
            var curveLimit = (newAngle < 0f) ? minAngle : maxAngle;
            newAngle = ComputeFalloff(newAngle, curveLimit) * curveLimit;

            var radians = Mathf.Deg2Rad * newAngle;
            var newProjection = Mathf.Sin(radians) * basisSide + Mathf.Cos(radians) * basisForward;
            newProjection *= projectionMagnitude;
            
            return newProjection + upProjection;
        }

        public static Vector3 Project(Vector3 vector, Vector3 onNormal) {
            float sqrMag = Vector3.Dot(onNormal, onNormal);
            // Burst対応の際にMathf.Epsilonを扱うと怒られる
            // Vector3.kEpsilonの1E-05は荒い気もするが元々Vector3演算なのでVector3の定義に合わせる
            //if (sqrMag < Mathf.Epsilon)
            if (sqrMag < Vector3.kEpsilon)
                return Vector3.zero;
            else {
                var dot = Vector3.Dot(vector, onNormal);
                return new Vector3(onNormal.x * dot / sqrMag,
                    onNormal.y * dot / sqrMag,
                    onNormal.z * dot / sqrMag);
            }
        }
    }
}