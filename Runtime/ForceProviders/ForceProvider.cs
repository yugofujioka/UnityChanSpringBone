﻿using Unity.Animations.SpringBones.Jobs;
using UnityEngine;

namespace Unity.Animations.SpringBones
{
    // スプリングボーン用の力を与えるベースクラス
    // FIXED: fujiioka 抽象クラスにしてForceVolumeと間違えないようにする
    public abstract class ForceProvider : MonoBehaviour
    {
        void OnEnable() {
            SpringJobScheduler.AddForceProvider(this);
        }
        void OnDisable() {
            SpringJobScheduler.RemoveForceProvider(this);
        }

        public virtual Vector3 GetForceOnBone(SpringBone springBone)
        {
            return Vector3.zero;
        }

        public virtual Jobs.SpringForceComponent GetActiveForce() {
            return default;
        }
    }
}

namespace Unity.Animations.SpringBones.Jobs {
    public enum SpringBoneForceType {
        Directional,
        Wind,
    }

    [System.Serializable]
    public struct SpringForceComponent {
        //public Matrix4x4 localToWorldMatrix;
        public Vector3 position;
        public Quaternion rotation;
        public SpringBoneForceType type;
        //public float weight;
        public float strength;
        public float amplitude;
        //public float periodInSecond;
        //public float spinPeriodInSecond;
        public float timeFactor;
        public float peakDistance;
        public Vector3 offsetVector;
    }
}