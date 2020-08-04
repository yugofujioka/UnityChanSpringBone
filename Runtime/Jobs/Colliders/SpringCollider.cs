using System;
using UnityEngine;

namespace Unity.Animations.SpringBones.Jobs
{
    // Authoring component
    public class SpringCollider : MonoBehaviour {
        public int layer;
        public ColliderType type;
        public float radius;
        public float width;
        public float height;

#if UNITY_EDITOR
        public bool shouldDrawGizmosThisFrame;

        public void DrawGizmos(Color drawColor) {
            var worldRadius = transform.TransformDirection(radius, 0f, 0f).magnitude;
            // For picking
            Gizmos.color = new Color(0f, 0f, 0f, 0f);
            Gizmos.DrawWireSphere(transform.position, worldRadius);

            UnityEditor.Handles.color = drawColor;
            UnityEditor.Handles.RadiusHandle(Quaternion.identity, transform.position, worldRadius);
            if (m_colliderDebugger != null) {
                m_colliderDebugger.DrawGizmosAndClear();
            }
        }

        private SpringManager manager;
        private SpringColliderDebugger m_colliderDebugger;

        private void OnDrawGizmos() {
            if (shouldDrawGizmosThisFrame || !SpringManager.onlyShowSelectedColliders) {
                if (manager == null) { manager = GetComponentInParent<SpringManager>(); }
                DrawGizmos((enabled && manager != null) ? manager.colliderColor : Color.gray);
                shouldDrawGizmosThisFrame = false;
            }
        }

        private void OnDrawGizmosSelected() {
            DrawGizmos(enabled ? Color.white : Color.gray);
        }

        private void RecordCollision
        (
            Vector3 localMoverPosition,
            float worldMoverRadius,
            SpringBone.CollisionStatus collisionStatus
        ) {
            if (!enabled) { return; }
            if (m_colliderDebugger == null) { m_colliderDebugger = new SpringColliderDebugger(); }
            var localNormal = (localMoverPosition).normalized;
            var localContactPoint = localNormal * radius;
            var worldNormal = transform.TransformDirection(localNormal).normalized;
            var worldContactPoint = transform.TransformPoint(localContactPoint);
            m_colliderDebugger.RecordCollision(worldContactPoint, worldNormal, worldMoverRadius, collisionStatus);
        }
#endif
    }
}